
/*
 * SPDX-License-Identifier: GPL-2.0
 * Copyright (c) Huawei Technologies Co., Ltd. 2024-2024. All rights reserved.
 * Kunpeng hardware accelerator queue management module.
 */

#include <linux/idr.h>
#include <linux/io.h>
#include <linux/uacce.h>
#include <uapi/misc/uacce/hisi_qm.h>
#include "plat_qm.h"

/* mailbox */
#define QM_MB_CMD_DATA_ADDR_L		0x304
#define QM_MB_CMD_DATA_ADDR_H		0x308
#define QM_MB_MAX_WAIT_CNT		6000
#define QM_MB_MAX_STOP_CNT		20000
#define QM_MB_WAIT_READY_CNT		10
#define QM_MB_STATUS_MASK		GENMASK(12, 9)
#define QM_MB_BUSY_MASK			BIT(13)
#define WAIT_PERIOD_US_MAX		200
#define WAIT_PERIOD_US_MIN		100
#define QM_MB_SIZE			16

/* cache */
#define POLL_PERIOD			10
#define POLL_TIMEOUT			1000
#define QM_CACHE_WB_START		0x204
#define QM_CACHE_WB_DONE		0x208

/* sqc */
#define QM_SQC_VFT_BASE_SHIFT		28
#define QM_SQC_VFT_BASE_MASK		GENMASK(15, 0)
#define QM_SQC_VFT_NUM_SHIFT		45
#define QM_SQC_VFT_NUM_MASK		GENMASK(9, 0)
#define QM_SQ_SQE_SIZE_SHIFT		12
#define QM_SQ_PRIORITY_SHIFT		0
#define QM_SQ_ORDERS_SHIFT		4
#define QM_SQ_TYPE_SHIFT		8

/* cqc */
#define QM_CQ_CQE_SIZE_SHIFT		12
#define QM_CQ_PHASE_SHIFT		0
#define QM_CQ_FLAG_SHIFT		1
#define QM_QC_CQE_SIZE			4
#define QM_CQE_PHASE(cqe)		(le16_to_cpu((cqe)->w7) & 0x1)

/* eq */
#define QM_EQ_DEPTH			(1024 * 2)
#define QM_EQC_PHASE_SHIFT		16

#define QM_RESET_STOP_TX_OFFSET		1

#define QM_MAX_QC_TYPE			2
#define QM_SQ_TYPE_MASK			GENMASK(3, 0)
#define QM_QP_DB_INTERVAL		0x10000

/* abnormal status value for stopping queue */
#define QM_STOP_QUEUE_FAIL		1

#define QMC_ALIGN(sz)			ALIGN(sz, 32)
#define QM_MK_SQC_DW3_V2(sqe_sz, sq_depth) \
	(((u32)(sq_depth) - 1) | ((u32)ilog2(sqe_sz) << QM_SQ_SQE_SIZE_SHIFT))
#define QM_MK_SQC_W13(priority, orders, alg_type) \
	(((priority) << QM_SQ_PRIORITY_SHIFT) | \
	((orders) << QM_SQ_ORDERS_SHIFT) | \
	(((alg_type) & QM_SQ_TYPE_MASK) << QM_SQ_TYPE_SHIFT))
#define QM_MK_CQC_DW3_V2(cqe_sz, cq_depth) \
	(((u32)(cq_depth) - 1) | ((cqe_sz) << QM_CQ_CQE_SIZE_SHIFT))

/* Platform device resource */
enum plat_resource_type {
	HAC_BAR_RESOURCE,
};

struct qm_mailbox {
	__le16 w0;
	__le16 queue_num;
	__le32 base_l;
	__le32 base_h;
	__le32 rsvd;
};

struct qm_hw_ops {
	int (*get_vft)(struct hisi_plat_qm *qm, u32 *base, u32 *number);
};

static void qm_mb_pre_init(struct qm_mailbox *mailbox, u8 cmd,
			   u64 base, u16 queue, bool op)
{
	mailbox->w0 = cpu_to_le16((cmd) | ((op) ? 0x1 << QM_MB_OP_SHIFT : 0) |
		      (0x1 << QM_MB_BUSY_SHIFT));
	mailbox->queue_num = cpu_to_le16(queue);
	mailbox->base_l = cpu_to_le32(lower_32_bits(base));
	mailbox->base_h = cpu_to_le32(upper_32_bits(base));
	mailbox->rsvd = 0;
}

/* 128 bit should be written to hardware at one time to trigger a mailbox */
static void qm_mb_write(struct hisi_plat_qm *qm, const void *src)
{
	void __iomem *fun_base = qm->io_base + QM_MB_CMD_SEND_BASE;

#if IS_ENABLED(CONFIG_ARM64)
	unsigned long tmp0 = 0, tmp1 = 0;
#endif

	if (!IS_ENABLED(CONFIG_ARM64)) {
		memcpy_toio(fun_base, src, QM_MB_SIZE);
		dma_wmb();
		return;
	}

#if IS_ENABLED(CONFIG_ARM64)
	asm volatile("ldp %0, %1, %3\n"
		     "stp %0, %1, %2\n"
		     "dmb oshst\n"
		     : "=&r" (tmp0),
		       "=&r" (tmp1),
		       "+Q" (*((char __iomem *)fun_base))
		     : "Q" (*((char *)src))
		     : "memory");
#endif
}

/* 128 bit should be read from hardware at one time */
static void qm_mb_read(struct hisi_plat_qm *qm, void *dst)
{
	const void __iomem *fun_base = qm->io_base + QM_MB_CMD_SEND_BASE;

#if IS_ENABLED(CONFIG_ARM64)
	unsigned long tmp0 = 0, tmp1 = 0;
#endif

	if (!IS_ENABLED(CONFIG_ARM64)) {
		memcpy_fromio(dst, fun_base, QM_MB_SIZE);
		dma_wmb();
		return;
	}

#if IS_ENABLED(CONFIG_ARM64)
	asm volatile("ldp %0, %1, %3\n"
		     "stp %0, %1, %2\n"
		     "dmb oshst\n"
		     : "=&r" (tmp0),
		       "=&r" (tmp1),
		       "+Q" (*((char *)dst))
		     : "Q" (*((char __iomem *)fun_base))
		     : "memory");
#endif
}

static int qm_wait_mb_ready(struct hisi_plat_qm *qm)
{
	struct qm_mailbox mailbox;
	int i = 0;

	while (i++ < QM_MB_WAIT_READY_CNT) {
		qm_mb_read(qm, &mailbox);
		if (!(le16_to_cpu(mailbox.w0) & QM_MB_BUSY_MASK))
			return 0;

		usleep_range(WAIT_PERIOD_US_MIN, WAIT_PERIOD_US_MAX);
	}

	dev_err(&qm->pdev->dev, "QM mailbox is busy to start!\n");

	return -EBUSY;
}

static int qm_wait_mb_finish(struct hisi_plat_qm *qm, struct qm_mailbox *mailbox, u32 wait_cnt)
{
	struct device *dev = &qm->pdev->dev;
	u32 i = 0;

	while (++i) {
		qm_mb_read(qm, mailbox);
		if (!(le16_to_cpu(mailbox->w0) & QM_MB_BUSY_MASK))
			break;

		if (i == wait_cnt) {
			dev_err(dev, "QM mailbox operation timeout!\n");
			return -ETIMEDOUT;
		}

		usleep_range(WAIT_PERIOD_US_MIN, WAIT_PERIOD_US_MAX);
	}

	if (le16_to_cpu(mailbox->w0) & QM_MB_STATUS_MASK) {
		dev_err(dev, "QM mailbox operation failed!\n");
		return -EIO;
	}

	return 0;
}

static int qm_mb_nolock(struct hisi_plat_qm *qm, struct qm_mailbox *mailbox, u32 wait_cnt)
{
	int ret;

	ret = qm_wait_mb_ready(qm);
	if (ret)
		goto mb_err_cnt_increase;

	qm_mb_write(qm, mailbox);

	ret = qm_wait_mb_finish(qm, mailbox, wait_cnt);
	if (ret)
		goto mb_err_cnt_increase;

	return 0;

mb_err_cnt_increase:
	atomic64_inc(&qm->debug.dfx.mb_err_cnt);
	return ret;
}

static int platform_qm_mb_write(struct hisi_plat_qm *qm, u8 cmd,
				dma_addr_t dma_addr, u16 queue, bool op)
{
	struct qm_mailbox mailbox;
	u32 wait_cnt;
	int ret;

	if (cmd == QM_MB_CMD_STOP_QP || cmd == QM_MB_CMD_FLUSH_QM)
		wait_cnt = QM_MB_MAX_STOP_CNT;
	else
		wait_cnt = QM_MB_MAX_WAIT_CNT;

	qm_mb_pre_init(&mailbox, cmd, dma_addr, queue, op);

	mutex_lock(&qm->mailbox_lock);
	ret = qm_mb_nolock(qm, &mailbox, wait_cnt);
	mutex_unlock(&qm->mailbox_lock);

	return ret;
}

static int platform_qm_mb_read(struct hisi_plat_qm *qm, u64 *base, u8 cmd,
			       u16 queue)
{
	struct qm_mailbox mailbox;
	int ret;

	qm_mb_pre_init(&mailbox, cmd, 0, queue, 1);
	mutex_lock(&qm->mailbox_lock);
	ret = qm_mb_nolock(qm, &mailbox, QM_MB_MAX_WAIT_CNT);
	mutex_unlock(&qm->mailbox_lock);
	if (ret)
		return ret;

	*base = le32_to_cpu(mailbox.base_l) |
		((u64)le32_to_cpu(mailbox.base_h) << 32);

	return 0;
}

/* op 0: set xqc information to hardware, 1: get xqc information from hardware. */
static int platform_qm_set_and_get_xqc(struct hisi_plat_qm *qm, u8 cmd,
				       void *xqc, u32 qp_id, bool op)
{
	struct qm_mailbox mailbox;
	dma_addr_t xqc_dma;
	void *tmp_xqc;
	size_t size;
	int ret;

	switch (cmd) {
	case QM_MB_CMD_SQC:
		size = sizeof(struct qm_sqc);
		tmp_xqc = qm->xqc_buf.sqc;
		xqc_dma = qm->xqc_buf.sqc_dma;
		break;
	case QM_MB_CMD_CQC:
		size = sizeof(struct qm_cqc);
		tmp_xqc = qm->xqc_buf.cqc;
		xqc_dma = qm->xqc_buf.cqc_dma;
		break;
	case QM_MB_CMD_EQC:
		size = sizeof(struct qm_eqc);
		tmp_xqc = qm->xqc_buf.eqc;
		xqc_dma = qm->xqc_buf.eqc_dma;
		break;
	case QM_MB_CMD_AEQC:
		size = sizeof(struct qm_aeqc);
		tmp_xqc = qm->xqc_buf.aeqc;
		xqc_dma = qm->xqc_buf.aeqc_dma;
		break;
	default:
		dev_err(&qm->pdev->dev, "unknown mailbox cmd %u\n", cmd);
		return -EINVAL;
	}

	mutex_lock(&qm->mailbox_lock);
	if (!op)
		memcpy(tmp_xqc, xqc, size);

	qm_mb_pre_init(&mailbox, cmd, xqc_dma, qp_id, op);
	ret = qm_mb_nolock(qm, &mailbox, QM_MB_MAX_WAIT_CNT);
	if (!ret && op)
		memcpy(xqc, tmp_xqc, size);

	mutex_unlock(&qm->mailbox_lock);

	return ret;
}

static int qm_get_vft(struct hisi_plat_qm *qm, u32 *base, u32 *number)
{
	u64 sqc_vft;
	int ret;

	ret = platform_qm_mb_read(qm, &sqc_vft, QM_MB_CMD_SQC_VFT_V2, 0);
	if (ret)
		return ret;

	*base = QM_SQC_VFT_BASE_MASK & (sqc_vft >> QM_SQC_VFT_BASE_SHIFT);
	*number = (QM_SQC_VFT_NUM_MASK & (sqc_vft >> QM_SQC_VFT_NUM_SHIFT)) + 1;

	return 0;
}

static const struct qm_hw_ops qm_hw_ops = {
	.get_vft = qm_get_vft,
};

static int qm_get_qp_num(struct hisi_plat_qm *qm)
{
	return qm->ops->get_vft(qm, &qm->qp_base, &qm->qp_num);
}

static int qm_get_res(struct hisi_plat_qm *qm)
{
	struct platform_device *pdev = qm->pdev;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, HAC_BAR_RESOURCE);
	if (!res) {
		dev_err(dev, "Fail to get platform resource!\n");
		return -EIO;
	}

	qm->phys_base = res->start;
	qm->io_base = ioremap(res->start, resource_size(res));
	if (!qm->io_base) {
		dev_err(dev, "Fail to ioremap io_base!\n");
		return -EIO;
	}

	ret = qm_get_qp_num(qm);
	if (ret)
		goto err_ioremap;

	return 0;

err_ioremap:
	iounmap(qm->io_base);
	return ret;
}

static void qm_put_res(struct hisi_plat_qm *qm)
{
	iounmap(qm->io_base);
}

static int qm_pre_init(struct hisi_plat_qm *qm)
{
	struct platform_device *pdev = qm->pdev;
	struct device *dev = &pdev->dev;
	int ret;

	qm->ops = &qm_hw_ops;
	platform_set_drvdata(pdev, qm);
	mutex_init(&qm->mailbox_lock);
	init_rwsem(&qm->qps_lock);
	qm->qp_in_used = 0;
	qm->use_uacce = true;

	ret = qm_get_res(qm);
	if (ret) {
		mutex_destroy(&qm->mailbox_lock);
		return ret;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret < 0) {
		dev_err(dev, "Fail to set 64 bit dma mask %d", ret);
		goto err_set_mask_and_coherent;
	}

	return 0;

err_set_mask_and_coherent:
	qm_put_res(qm);
	mutex_destroy(&qm->mailbox_lock);
	return ret;
}

static int qp_memory_init(struct hisi_plat_qm *qm, size_t dma_size, int id)
{
	struct device *dev = &qm->pdev->dev;
	size_t off = qm->sqe_size * QM_Q_DEPTH;
	struct hisi_plat_qp *qp;

	qp = &qm->qp_array[id];
	qp->qdma.va = dma_alloc_coherent(dev, dma_size, &qp->qdma.dma, GFP_KERNEL);
	if (!qp->qdma.va) {
		dev_err(dev, "Fail to alloc coherent dma!\n");
		return -ENOMEM;
	}

	qp->sqe = qp->qdma.va;
	qp->sqe_dma = qp->qdma.dma;
	qp->cqe = qp->qdma.va + off;
	qp->cqe_dma = qp->qdma.dma + off;
	qp->qdma.size = dma_size;
	qp->qm = qm;
	qp->qp_id = id;

	return 0;
}

static void qp_memory_uninit(struct hisi_plat_qm *qm, int num)
{
	struct device *dev = &qm->pdev->dev;
	struct qm_dma *qdma;
	int i;

	for (i = num - 1; i >= 0; i--) {
		qdma = &qm->qp_array[i].qdma;
		dma_free_coherent(dev, qdma->size, qdma->va, qdma->dma);
	}

	kfree(qm->qp_array);
}

static int qp_alloc_memory(struct hisi_plat_qm *qm)
{
	struct device *dev = &qm->pdev->dev;
	size_t qp_dma_size;
	int i, ret;

	qm->qp_array = kcalloc(qm->qp_num, sizeof(struct hisi_plat_qp), GFP_KERNEL);
	if (!qm->qp_array) {
		dev_err(dev, "Fail to kcalloc qp_array!\n");
		return -ENOMEM;
	}

	/* one more page for device or qp statuses */
	qp_dma_size = qm->sqe_size * QM_Q_DEPTH + sizeof(struct qm_cqe) * QM_Q_DEPTH;
	qp_dma_size = PAGE_ALIGN(qp_dma_size) + PAGE_SIZE;
	for (i = 0; i < qm->qp_num; i++) {
		ret = qp_memory_init(qm, qp_dma_size, i);
		if (ret)
			goto err_init_qp_mem;

		dev_dbg(dev, "allocate qp dma buf size=%zx)\n", qp_dma_size);
	}

	return 0;
err_init_qp_mem:
	qp_memory_uninit(qm, i);

	return ret;
}

static int qm_alloc_rsv_buf(struct hisi_plat_qm *qm)
{
	struct qm_rsv_buf *xqc_buf = &qm->xqc_buf;
	struct qm_dma *xqc_dma = &xqc_buf->qcdma;
	struct device *dev = &qm->pdev->dev;
	size_t off = 0;

#define QM_XQC_BUF_INIT(xqc_buf, type) do { \
	(xqc_buf)->type = ((xqc_buf)->qcdma.va + (off)); \
	(xqc_buf)->type##_dma = (xqc_buf)->qcdma.dma + (off); \
	off += QMC_ALIGN(sizeof(struct qm_##type)); \
} while (0)

	xqc_dma->size = QMC_ALIGN(sizeof(struct qm_eqc)) +
			QMC_ALIGN(sizeof(struct qm_aeqc)) +
			QMC_ALIGN(sizeof(struct qm_sqc)) +
			QMC_ALIGN(sizeof(struct qm_cqc));
	xqc_dma->va = dma_alloc_coherent(dev, xqc_dma->size,
					 &xqc_dma->dma, GFP_KERNEL);
	if (!xqc_dma->va) {
		dev_err(dev, "Fail to alloc coherent dma!\n");
		return -ENOMEM;
	}

	QM_XQC_BUF_INIT(xqc_buf, eqc);
	QM_XQC_BUF_INIT(xqc_buf, aeqc);
	QM_XQC_BUF_INIT(xqc_buf, sqc);
	QM_XQC_BUF_INIT(xqc_buf, cqc);

	return 0;
}

static void qm_free_rsv_buf(struct hisi_plat_qm *qm)
{
	struct qm_dma *xqc_dma = &qm->xqc_buf.qcdma;
	struct device *dev = &qm->pdev->dev;

	dma_free_coherent(dev, xqc_dma->size, xqc_dma->va, xqc_dma->dma);
}

static int qm_memory_init(struct hisi_plat_qm *qm)
{
	struct device *dev = &qm->pdev->dev;
	size_t off = 0;
	int ret;

#define QM_INIT_BUF(qm, type, num) do { \
	(qm)->type = ((qm)->qdma.va + (off)); \
	(qm)->type##_dma = (qm)->qdma.dma + (off); \
	off += QMC_ALIGN(sizeof(struct qm_##type) * (num)); \
} while (0)

	idr_init(&qm->qp_idr);
	qm->qdma.size = QMC_ALIGN(sizeof(struct qm_eqe) * QM_EQ_DEPTH) +
			QMC_ALIGN(sizeof(struct qm_aeqe) * QM_Q_DEPTH) +
			QMC_ALIGN(sizeof(struct qm_sqc) * qm->qp_num) +
			QMC_ALIGN(sizeof(struct qm_cqc) * qm->qp_num);
	qm->qdma.va = dma_alloc_coherent(dev, qm->qdma.size, &qm->qdma.dma, GFP_ATOMIC);
	if (!qm->qdma.va) {
		ret = -ENOMEM;
		goto err_destroy_idr;
	}

	QM_INIT_BUF(qm, eqe, QM_EQ_DEPTH);
	QM_INIT_BUF(qm, aeqe, QM_Q_DEPTH);
	QM_INIT_BUF(qm, sqc, qm->qp_num);
	QM_INIT_BUF(qm, cqc, qm->qp_num);

	ret = qm_alloc_rsv_buf(qm);
	if (ret)
		goto err_free_qdma;

	ret = qp_alloc_memory(qm);
	if (ret)
		goto err_free_reserve_buf;

	return 0;

err_free_reserve_buf:
	qm_free_rsv_buf(qm);
err_free_qdma:
	dma_free_coherent(dev, qm->qdma.size, qm->qdma.va, qm->qdma.dma);
err_destroy_idr:
	idr_destroy(&qm->qp_idr);

	return ret;
}

/* This function returns free number of qp in qm. */
static int qm_get_available_instances(struct uacce_device *uacce)
{
	struct hisi_plat_qm *qm = uacce->priv;
	int ret;

	down_read(&qm->qps_lock);
	ret = qm->qp_num - qm->qp_in_used;
	up_read(&qm->qps_lock);

	return ret;
}

static void qm_unset_hw_reset(struct hisi_plat_qp *qp)
{
	u64 *addr;

	/* Use last 64 bits of DUS to reset status. */
	addr = (u64 *)(qp->qdma.va + qp->qdma.size) - QM_RESET_STOP_TX_OFFSET;
	*addr = 0;
}

static struct hisi_plat_qp *qm_create_qp_nolock(struct hisi_plat_qm *qm,
						u8 alg_type)
{
	struct device *dev = &qm->pdev->dev;
	struct hisi_plat_qp *qp;
	int qp_id;

	if (atomic_read(&qm->status.flags) == QM_STOP) {
		dev_info_ratelimited(dev, "fail to create qp as qm is stop!\n");
		return ERR_PTR(-EPERM);
	}

	if (qm->qp_in_used == qm->qp_num) {
		dev_info_ratelimited(dev, "All %u queues of QM are busy!\n",
				     qm->qp_num);
		atomic64_inc(&qm->debug.dfx.create_qp_err_cnt);
		return ERR_PTR(-EBUSY);
	}

	qp_id = idr_alloc_cyclic(&qm->qp_idr, NULL, 0, qm->qp_num, GFP_ATOMIC);
	if (qp_id < 0) {
		dev_info_ratelimited(dev, "All %u queues of QM are busy!\n",
				    qm->qp_num);
		atomic64_inc(&qm->debug.dfx.create_qp_err_cnt);
		return ERR_PTR(-EBUSY);
	}

	qp = &qm->qp_array[qp_id];
	qm_unset_hw_reset(qp);
	memset(qp->cqe, 0, sizeof(struct qm_cqe) * QM_Q_DEPTH);

	qp->alg_type = alg_type;
	qp->is_in_kernel = true;
	qm->qp_in_used++;

	return qp;
}

/**
 * qm_create_qp() - Create a queue pair from qm.
 * @qm: The qm we create a qp from.
 * @alg_type: Accelerator specific algorithm type in sqc.
 *
 * Return created qp, negative error code if failed.
 */
static struct hisi_plat_qp *qm_create_qp(struct hisi_plat_qm *qm, u8 alg_type)
{
	struct hisi_plat_qp *qp;

	down_write(&qm->qps_lock);
	qp = qm_create_qp_nolock(qm, alg_type);
	up_write(&qm->qps_lock);

	return qp;
}

static int qm_uacce_get_queue(struct uacce_device *uacce, unsigned long arg,
			      struct uacce_queue *q)
{
	struct hisi_plat_qm *qm = uacce->priv;
	struct hisi_plat_qp *qp;
	u8 alg_type = 0;

	qp = qm_create_qp(qm, alg_type);
	if (IS_ERR(qp))
		return PTR_ERR(qp);

	q->priv = qp;
	q->uacce = uacce;
	qp->uacce_q = q;
	qp->pasid = arg;
	qp->is_in_kernel = false;

	return 0;
}

/**
 * qm_release_qp() - Release a qp back to its qm.
 * @qp: The qp we want to release.
 *
 * This function releases the resource of a qp.
 */
static void qm_release_qp(struct hisi_plat_qp *qp)
{
	struct hisi_plat_qm *qm = qp->qm;

	down_write(&qm->qps_lock);

	qm->qp_in_used--;
	idr_remove(&qm->qp_idr, qp->qp_id);

	up_write(&qm->qps_lock);
}

static void qm_uacce_put_queue(struct uacce_queue *q)
{
	struct hisi_plat_qp *qp = q->priv;

	qm_release_qp(qp);
}

static void qm_init_qp_status(struct hisi_plat_qp *qp)
{
	struct hisi_qp_status *qp_status = &qp->qp_status;

	qp_status->sq_tail = 0;
	qp_status->cq_head = 0;
	qp_status->cqc_phase = true;
	atomic_set(&qp_status->used, 0);
}

static int qm_sq_ctx_cfg(struct hisi_plat_qp *qp, int qp_id, u32 pasid)
{
	struct hisi_plat_qm *qm = qp->qm;
	struct qm_sqc sqc = {0};

	sqc.dw3 = cpu_to_le32(QM_MK_SQC_DW3_V2(qm->sqe_size, QM_Q_DEPTH));

	sqc.w13 = cpu_to_le16(QM_MK_SQC_W13(0, 1, qp->alg_type));
	sqc.base_l = cpu_to_le32(lower_32_bits(qp->sqe_dma));
	sqc.base_h = cpu_to_le32(upper_32_bits(qp->sqe_dma));
	sqc.cq_num = cpu_to_le16(qp_id);
	sqc.w8 = 0; /* rand_qc */
	sqc.pasid = cpu_to_le16(pasid);

	return platform_qm_set_and_get_xqc(qm, QM_MB_CMD_SQC, &sqc, qp_id, 0);
}

static int qm_cq_ctx_cfg(struct hisi_plat_qp *qp, int qp_id, u32 pasid)
{
	struct hisi_plat_qm *qm = qp->qm;
	struct qm_cqc cqc = {0};

	/*
	 * Enable request finishing interrupts defaultly.
	 * So, there will be some interrupts until disabling
	 * this.
	 */
	cqc.dw6 = cpu_to_le32(1 << QM_CQ_PHASE_SHIFT);
	cqc.base_l = cpu_to_le32(lower_32_bits(qp->cqe_dma));
	cqc.base_h = cpu_to_le32(upper_32_bits(qp->cqe_dma));
	cqc.dw3 = cpu_to_le32(QM_MK_CQC_DW3_V2(QM_QC_CQE_SIZE, QM_Q_DEPTH));
	cqc.w8 = 0; /* rand_qc */
	cqc.pasid = cpu_to_le16(pasid);

	return platform_qm_set_and_get_xqc(qm, QM_MB_CMD_CQC, &cqc, qp_id, 0);
}

static int qm_qp_ctx_cfg(struct hisi_plat_qp *qp, int qp_id, u32 pasid)
{
	int ret;

	qm_init_qp_status(qp);

	ret = qm_sq_ctx_cfg(qp, qp_id, pasid);
	if (ret)
		return ret;

	return qm_cq_ctx_cfg(qp, qp_id, pasid);
}

static int qm_start_qp_nolock(struct hisi_plat_qp *qp, unsigned long arg)
{
	struct hisi_plat_qm *qm = qp->qm;
	struct device *dev = &qm->pdev->dev;
	int qp_id = qp->qp_id;
	u32 pasid = arg;
	int ret;

	if (atomic_read(&qm->status.flags) == QM_STOP) {
		dev_info_ratelimited(dev, "fail to start qp as qm is stop!\n");
		return -EPERM;
	}

	ret = qm_qp_ctx_cfg(qp, qp_id, pasid);
	if (ret)
		return ret;

	atomic_set(&qp->qp_status.flags, QP_START);
	dev_dbg(dev, "queue %d started\n", qp_id);

	return 0;
}

/**
 * qm_start_qp() - Start a qp into running.
 * @qp: The qp we want to start to run.
 * @arg: Accelerator specific argument.
 *
 * After this function, qp can receive request from user. Return 0 if
 * successful, negative error code if failed.
 */
static int qm_start_qp(struct hisi_plat_qp *qp, unsigned long arg)
{
	struct hisi_plat_qm *qm = qp->qm;
	int ret;

	down_write(&qm->qps_lock);
	ret = qm_start_qp_nolock(qp, arg);
	up_write(&qm->qps_lock);

	return ret;
}

static int qm_uacce_start_queue(struct uacce_queue *q)
{
	struct hisi_plat_qp *qp = q->priv;

	return qm_start_qp(qp, qp->pasid);
}

static int qm_stop_qp(struct hisi_plat_qp *qp)
{
	return platform_qm_mb_write(qp->qm, QM_MB_CMD_STOP_QP, 0, qp->qp_id, 0);
}

/**
 * qm_drain_qp() - Drain a qp.
 * @qp: The qp we want to drain.
 *
 * If the device does not support stopping queue by sending mailbox,
 * determine whether the queue is cleared by judging the tail pointers of
 * sq and cq.
 */
static int qm_drain_qp(struct hisi_plat_qp *qp)
{
	struct hisi_plat_qm *qm = qp->qm;
	u32 state = 0;
	int ret;

	ret = qm_stop_qp(qp);
	if (ret) {
		dev_err(&qm->pdev->dev, "Fail to stop qp!\n");
		state = QM_STOP_QUEUE_FAIL;
		goto set_dev_state;
	}

	return 0;

set_dev_state:
	if (qm->debug.dev_dfx.dev_timeout)
		qm->debug.dev_dfx.dev_state = state;

	return ret;
}

static void qm_stop_qp_nolock(struct hisi_plat_qp *qp)
{
	struct hisi_plat_qm *qm = qp->qm;
	struct device *dev = &qm->pdev->dev;
	int ret;

	if (atomic_read(&qp->qp_status.flags) != QP_START)
		return;

	atomic_set(&qp->qp_status.flags, QP_STOP);
	if (qm->status.stop_reason == QM_NORMAL) {
		ret = qm_drain_qp(qp);
		if (ret)
			dev_err(dev, "Failed to drain out data for stopping qp(%u)!\n", qp->qp_id);
	}
	dev_dbg(dev, "stop queue %u!", qp->qp_id);
}

static void qm_uacce_stop_queue(struct uacce_queue *q)
{
	struct hisi_plat_qp *qp = q->priv;

	down_write(&qp->qm->qps_lock);
	qm_stop_qp_nolock(qp);
	up_write(&qp->qm->qps_lock);
}

/* map sq/cq/doorbell to user space */
static int qm_uacce_mmap(struct uacce_queue *q, struct vm_area_struct *vma,
			 struct uacce_qfile_region *qfr)
{
	struct hisi_plat_qp *qp = q->priv;
	struct hisi_plat_qm *qm = qp->qm;
	resource_size_t phys_base = qm->phys_base;
	size_t sz = vma->vm_end - vma->vm_start;
	struct device *dev = &qm->pdev->dev;
	unsigned long vm_pgoff;
	int ret;

	switch (qfr->type) {
	case UACCE_QFRT_MMIO:
		if (sz > PAGE_SIZE * (QM_DOORBELL_PAGE_NR +
			    QM_DOORBELL_SQ_CQ_BASE_V2 / PAGE_SIZE)) {
			dev_err(dev, "Invalid Map Size=0x%lx\n", sz);
			return -EINVAL;
		}
		vm_flags_set(vma, VM_IO);
		return remap_pfn_range(vma, vma->vm_start,
				       phys_base >> PAGE_SHIFT,
				       sz, pgprot_noncached(vma->vm_page_prot));
	case UACCE_QFRT_DUS:
		if (sz != qp->qdma.size) {
			dev_err(dev, "wrong queue size %ld vs %ld\n",
				sz, qp->qdma.size);
			return -EINVAL;
		}

		/*
		 * dma_mmap_coherent() requires vm_pgoff as 0
		 * restore vm_pfoff to initial value for mmap()
		 */
		vm_pgoff = vma->vm_pgoff;
		vma->vm_pgoff = 0;
		ret = dma_mmap_coherent(dev, vma, qp->qdma.va,
					qp->qdma.dma, sz);
		vma->vm_pgoff = vm_pgoff;
		return ret;

	default:
		return -EINVAL;
	}
}

static void qm_set_sqctype(struct uacce_queue *q, u16 type)
{
	struct hisi_plat_qm *qm = q->uacce->priv;
	struct hisi_plat_qp *qp = q->priv;

	down_write(&qm->qps_lock);
	qp->alg_type = type;
	up_write(&qm->qps_lock);
}

static long qm_uacce_ioctl(struct uacce_queue *q, unsigned int cmd,
			   unsigned long arg)
{
	struct hisi_plat_qp *qp = q->priv;
	struct hisi_qp_info qp_info;
	struct hisi_qp_ctx qp_ctx;

	if (!access_ok((void __user *)arg, sizef(struct hisi_qp_ctx)))
		return -EINVAL;

	if (cmd == UACCE_CMD_QM_SET_QP_CTX) {
		if (copy_from_user(&qp_ctx, (void __user *)arg,
				   sizeof(struct hisi_qp_ctx)))
			return -EFAULT;

		if (qp_ctx.qc_type > QM_MAX_QC_TYPE)
			return -EINVAL;

		qm_set_sqctype(q, qp_ctx.qc_type);
		qp_ctx.id = qp->qp_id;

		if (copy_to_user((void __user *)arg, &qp_ctx,
				 sizeof(struct hisi_qp_ctx)))
			return -EFAULT;

		return 0;
	} else if (cmd == UACCE_CMD_QM_SET_QP_INFO) {
		if (copy_from_user(&qp_info, (void __user *)arg,
				   sizeof(struct hisi_qp_info)))
			return -EFAULT;

		qp_info.sqe_size = qp->qm->sqe_size;
		qp_info.sq_depth = QM_Q_DEPTH;
		qp_info.cq_depth = QM_Q_DEPTH;

		if (copy_to_user((void __user *)arg, &qp_info,
				  sizeof(struct hisi_qp_info)))
			return -EFAULT;

		return 0;
	}

	return -EINVAL;
}

static enum uacce_dev_state qm_get_isolate_state(struct uacce_device *uacce)
{
	/* UACCE_DEV_ISOLATE is not supported for platform vf */
	return UACCE_DEV_NORMAL;
}

static int qm_isolate_threshold_write(struct uacce_device *uacce, u32 num)
{
	/* Must be set by PF */
	if (uacce->is_vf)
		return -EPERM;

	return 0;
}

static u32 qm_isolate_threshold_read(struct uacce_device *uacce)
{
	/* UACCE_DEV_ISOLATE is not supported for platform vf */
	return 0;
}

static const struct uacce_ops uacce_qm_ops = {
	.get_available_instances = qm_get_available_instances,
	.get_queue = qm_uacce_get_queue,
	.put_queue = qm_uacce_put_queue,
	.start_queue = qm_uacce_start_queue,
	.stop_queue = qm_uacce_stop_queue,
	.mmap = qm_uacce_mmap,
	.ioctl = qm_uacce_ioctl,
	.get_isolate_state = qm_get_isolate_state,
	.isolate_err_threshold_write = qm_isolate_threshold_write,
	.isolate_err_threshold_read = qm_isolate_threshold_read,
};

static void qm_uacce_base_init(struct hisi_plat_qm *qm)
{
	unsigned long dus_page_nr, mmio_page_nr;
	struct platform_device *pdev = qm->pdev;
	struct uacce_device *uacce = qm->uacce;

	mmio_page_nr = QM_DOORBELL_PAGE_NR + QM_DOORBELL_SQ_CQ_BASE_V2 / PAGE_SIZE;
	uacce->is_vf = true;
	uacce->priv = qm;
	uacce->algs = qm->algs;
	uacce->parent = &pdev->dev;
	uacce->api_ver = HISI_QM_API_VER3_BASE UACCE_API_VER_NOIOMMU_SUBFIX;

	/* Add one more page for device or qp status */
	dus_page_nr = (PAGE_SIZE - 1 + qm->sqe_size * QM_Q_DEPTH +
		       sizeof(struct qm_cqe) * QM_Q_DEPTH + PAGE_SIZE) >>
		      PAGE_SHIFT;

	uacce->qf_pg_num[UACCE_QFRT_MMIO] = mmio_page_nr;
	uacce->qf_pg_num[UACCE_QFRT_DUS] = dus_page_nr;
}

static inline bool is_iommu_used(struct device *dev)
{
	struct iommu_domain *domain;

	domain = iommu_get_domain_for_dev(dev);
	if (domain)
		if (domain->type & __IOMMU_DOMAIN_PAGING)
			return true;

	return false;
}

static int qm_alloc_uacce(struct hisi_plat_qm *qm)
{
	struct platform_device *pdev = qm->pdev;
	struct uacce_interface interface = {0};
	struct device *dev = &qm->pdev->dev;
	struct uacce_device *uacce;
	u32 flags;
	int ret;

	ret = strscpy(interface.name, dev_driver_string(&pdev->dev),
		      sizeof(interface.name));
	if (ret < 0)
		return -ENAMETOOLONG;

	if (is_iommu_used(dev))
		interface.flags = UACCE_DEV_IOMMU;
	else
		interface.flags = UACCE_DEV_NOIOMMU;

	interface.ops = &uacce_qm_ops;
	flags = interface.flags;
	uacce = uacce_alloc(&pdev->dev, &interface);
	if (IS_ERR(uacce)) {
		dev_err(dev, "fail to alloc uacce device!\n");
		return PTR_ERR(uacce);
	}

	qm->uacce = uacce;

	qm_uacce_base_init(qm);

	return 0;
}

static void qm_remove_uacce(struct hisi_plat_qm *qm)
{
	struct uacce_device *uacce = qm->uacce;

	if (uacce) {
		uacce_remove(uacce);
		qm->uacce = NULL;
	}
}

/**
 * platform_init_qm() - Initialize configures about qm.
 * @qm: The qm needing init.
 *
 * This function init qm, then we can call platform_start_qm to put qm into work.
 */
int platform_init_qm(struct hisi_plat_qm *qm)
{
	int ret;

	if (!qm)
		return -EINVAL;

	ret = qm_pre_init(qm);
	if (ret)
		return ret;

	ret = qm_alloc_uacce(qm);
	if (ret < 0) {
		dev_err(&qm->pdev->dev, "fail to alloc uacce (%d)\n", ret);
		goto err_pre_init;
	}

	ret = qm_memory_init(qm);
	if (ret)
		goto err_alloc_uacce;

	return 0;

err_alloc_uacce:
	qm_remove_uacce(qm);

err_pre_init:
	qm_put_res(qm);
	mutex_destroy(&qm->mailbox_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(platform_init_qm);

static int qm_eq_ctx_cfg(struct hisi_plat_qm *qm)
{
	struct qm_eqc eqc = {0};

	eqc.base_l = cpu_to_le32(lower_32_bits(qm->eqe_dma));
	eqc.base_h = cpu_to_le32(upper_32_bits(qm->eqe_dma));
	eqc.dw6 = cpu_to_le32((QM_EQ_DEPTH - 1) | (1 << QM_EQC_PHASE_SHIFT));

	return platform_qm_set_and_get_xqc(qm, QM_MB_CMD_EQC, &eqc, 0, 0);
}

static int qm_aeq_ctx_cfg(struct hisi_plat_qm *qm)
{
	struct qm_aeqc aeqc = {0};

	aeqc.base_l = cpu_to_le32(lower_32_bits(qm->aeqe_dma));
	aeqc.base_h = cpu_to_le32(upper_32_bits(qm->aeqe_dma));
	aeqc.dw6 = cpu_to_le32((QM_Q_DEPTH - 1) | (1 << QM_EQC_PHASE_SHIFT));

	return platform_qm_set_and_get_xqc(qm, QM_MB_CMD_AEQC, &aeqc, 0, 0);
}

static int qm_eq_aeq_ctx_cfg(struct hisi_plat_qm *qm)
{
	struct device *dev = &qm->pdev->dev;
	int ret;

	ret = qm_eq_ctx_cfg(qm);
	if (ret) {
		dev_err(dev, "Fail to set eqc!\n");
		return ret;
	}

	return qm_aeq_ctx_cfg(qm);
}

static int qm_dev_start(struct hisi_plat_qm *qm)
{
	int ret;

	WARN_ON(!qm->qdma.dma);

	ret = qm_eq_aeq_ctx_cfg(qm);
	if (ret)
		return ret;

	ret = platform_qm_mb_write(qm, QM_MB_CMD_SQC_BT, qm->sqc_dma, 0, 0);
	if (ret)
		return ret;

	ret = platform_qm_mb_write(qm, QM_MB_CMD_CQC_BT, qm->cqc_dma, 0, 0);
	if (ret)
		return ret;

	return 0;
}

static void qm_set_state(struct hisi_plat_qm *qm, u8 state)
{
	writel(state, qm->io_base + QM_VF_STATE);
}

/**
 * platform_start_qm() - start qm
 * @qm: The qm to be started.
 *
 * This function starts a qm.
 */
int platform_start_qm(struct hisi_plat_qm *qm)
{
	struct device *dev = &qm->pdev->dev;
	int ret = 0;

	down_write(&qm->qps_lock);

	dev_info(dev, "qm start with %lu queue pairs\n", qm->qp_num);

	if (!qm->qp_num) {
		dev_err(dev, "qp_num should not be 0\n");
		ret = -EINVAL;
		goto err_unlock;
	}

	ret = qm_dev_start(qm);
	if (ret)
		goto err_unlock;

	atomic_set(&qm->status.flags, QM_WORK);
	qm_set_state(qm, QM_READY);

err_unlock:
	up_write(&qm->qps_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(platform_start_qm);

/**
 * This function clears all queues memory in a qm. Reset of accelerator can
 * use this to clear queues.
 */
static void qm_clear_queues(struct hisi_plat_qm *qm)
{
	struct hisi_plat_qp *qp;
	int i;

	for (i = 0; i < qm->qp_num; i++) {
		qp = &qm->qp_array[i];
		/* device state use the last page */
		memset(qp->qdma.va, 0, qp->qdma.size);
	}

	memset(qm->qdma.va, 0, qm->qdma.size);
}

/**
 * platform_stop_qm() - Stop a qm.
 * @qm: The qm which will be stopped.
 * @r: The reason to stop qm.
 *
 * This function stops qm and its qps, then qm can not accept request.
 * Related resources are not released at this state, we can use platform_start_qm
 * to let qm start again.
 */
int platform_stop_qm(struct hisi_plat_qm *qm, enum qm_stop_reason r)
{
	down_write(&qm->qps_lock);

	if (atomic_read(&qm->status.flags) == QM_STOP)
		goto err_unlock;

	/* stop all the request sending at first. */
	atomic_set(&qm->status.flags, QM_STOP);

	qm_clear_queues(qm);
	qm->status.stop_reason = r;

err_unlock:
	up_write(&qm->qps_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(platform_stop_qm);

static void qm_cache_wb(struct hisi_plat_qm *qm)
{
	unsigned int val;

	writel(0x1, qm->io_base + QM_CACHE_WB_START);
	if (readl_relaxed_poll_timeout(qm->io_base + QM_CACHE_WB_DONE,
						val, val & BIT(0), POLL_PERIOD,
						POLL_TIMEOUT))
		dev_err(&qm->pdev->dev, "QM writeback sqc cache fail!\n");
}

static void qm_memory_uninit(struct hisi_plat_qm *qm)
{
	struct platform_device *pdev = qm->pdev;
	struct device *dev = &pdev->dev;

	qp_memory_uninit(qm, qm->qp_num);
	qm_free_rsv_buf(qm);
	if (qm->qdma.va) {
		qm_cache_wb(qm);
		dma_free_coherent(dev, qm->qdma.size,
				  qm->qdma.va, qm->qdma.dma);
	}

	idr_destroy(&qm->qp_idr);
}

/**
 * platform_uninit_qm() - Uninitialize qm.
 * @qm: The qm needed uninit.
 *
 * This function uninits qm related device resources.
 */
void platform_uninit_qm(struct hisi_plat_qm *qm)
{
	down_write(&qm->qps_lock);
	qm_memory_uninit(qm);
	qm_set_state(qm, QM_NOT_READY);
	up_write(&qm->qps_lock);
	qm_remove_uacce(qm);
	qm_put_res(qm);
}
EXPORT_SYMBOL_GPL(platform_uninit_qm);

int platform_register_uacce(struct hisi_plat_qm *qm)
{
	if (!qm)
		return -EINVAL;

	if (!qm->uacce)
		return 0;

	dev_info(&qm->pdev->dev, "qm register to uacce\n");
	return uacce_register(qm->uacce);
}
EXPORT_SYMBOL_GPL(platform_register_uacce);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Leisen <leisen1@huawei.com>");
MODULE_AUTHOR("Guangwei Zhou <zhouguangwei5@huawei.com>");
MODULE_AUTHOR("Wangyuan <wangyuan46@huawei.com>");
MODULE_DESCRIPTION("HiSilicon Accelerator queue manager Platform driver");
