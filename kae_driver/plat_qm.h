
/*
 * SPDX-License-Identifier: GPL-2.0
 * Copyright (c) Huawei Technologies Co., Ltd. 2024-2024. All rights reserved.
 */

#ifndef PLAT_QM_H
#define PLAT_QM_H

#include <linux/hisi_acc_qm.h>
#include <linux/platform_device.h>

#define QM_Q_DEPTH			1024

struct hisi_plat_qp {
	u32 qp_id;
	u16 sq_depth;
	u16 cq_depth;
	u8 alg_type;
	u8 req_type;

	struct qm_dma qdma;
	void *sqe;
	struct qm_cqe *cqe;
	dma_addr_t sqe_dma;
	dma_addr_t cqe_dma;

	struct hisi_qp_status qp_status;
	struct hisi_qp_ops *hw_ops;
	void *qp_ctx;

	struct hisi_plat_qm *qm;
	bool is_in_kernel;
	u16 pasid;
	struct uacce_queue *uacce_q;
};

struct hisi_plat_qm {
	enum qm_hw_ver ver;
	const char *dev_name;
	struct platform_device *pdev;
	void __iomem *io_base;
	u32 sqe_size;
	u32 qp_base;
	u32 max_qp_num;
	u32 qp_num;
	u32 qp_in_used;
	struct list_head list;

	struct qm_dma qdma;
	struct qm_sqc *sqc;
	struct qm_cqc *cqc;
	struct qm_eqe *eqe;
	struct qm_aeqe *aeqe;
	dma_addr_t sqc_dma;
	dma_addr_t cqc_dma;
	dma_addr_t eqe_dma;
	dma_addr_t aeqe_dma;
	struct qm_rsv_buf xqc_buf;

	struct hisi_qm_status status;

	struct rw_semaphore qps_lock;
	struct idr qp_idr;
	struct hisi_plat_qp *qp_array;
	struct mutex mailbox_lock;
	const struct qm_hw_ops *ops;
	struct qm_debug debug;

	bool use_uacce;        /* register to uacce */

	resource_size_t phys_base;
	struct uacce_device *uacce;
	const char *algs;
};

int platform_init_qm(struct hisi_plat_qm *qm);
int platform_start_qm(struct hisi_plat_qm *qm);
int platform_stop_qm(struct hisi_plat_qm *qm, enum qm_stop_reason r);
void platform_uninit_qm(struct hisi_plat_qm *qm);
int platform_register_uacce(struct hisi_plat_qm *qm);
#endif /* !PLAT_QM_H */