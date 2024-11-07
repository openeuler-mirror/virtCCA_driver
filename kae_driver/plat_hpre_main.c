
/*
 * SPDX-License-Identifier: GPL-2.0
 * Copyright (c) Huawei Technologies Co., Ltd. 2024-2024. All rights reserved.
 * Kunpeng hardware accelerator hpre module init.
 */

#include <linux/init.h>
#include <linux/module.h>

#include "plat_qm.h"

struct plat_hpre {
	struct hisi_plat_qm qm;
};

#define HPRE_SQE_SIZE		64

static int hpre_qm_init(struct hisi_plat_qm *qm, struct platform_device *pdev)
{
	qm->algs = "rsa\ndh\necdh\nx25519\nx448\necdsa\nsm2\n";
	qm->sqe_size = HPRE_SQE_SIZE;
	qm->pdev = pdev;

	return platform_init_qm(qm);
}

static int hpre_probe(struct platform_device *pdev)
{
	struct hisi_plat_qm *qm = NULL;
	struct plat_hpre *hpre = NULL;
	int ret = 0;

	hpre = devm_kzalloc(&pdev->dev, sizeof(*hpre), GFP_KERNEL);
	if (!hpre) {
		dev_err(&pdev->dev, "Fail to alloc hpre!\n");
		return -ENOMEM;
	}

	qm = &hpre->qm;
	ret = hpre_qm_init(qm, pdev);
	if (ret) {
		dev_err(&pdev->dev, "Fail to init hpre qm (%d)!\n", ret);
		return ret;
	}

	ret = platform_start_qm(qm);
	if (ret) {
		dev_err(&pdev->dev, "Fail to start hpre qm (%d)!\n", ret);
		goto err_with_qm_init;
	}

	ret = platform_register_uacce(qm);
	if (ret) {
		dev_err(&pdev->dev, "Fail to register uacce (%d)!\n", ret);
		goto err_with_qm_start;
	}

	dev_info(&pdev->dev, "HPRE PF probe success!\n");
	return 0;

err_with_qm_start:
	platform_stop_qm(qm, QM_NORMAL);

err_with_qm_init:
	platform_uninit_qm(qm);

	return ret;
}

static int hpre_remove(struct platform_device *pdev)
{
	struct hisi_plat_qm *qm = platform_get_drvdata(pdev);

	(void)platform_stop_qm(qm, QM_NORMAL);
	platform_uninit_qm(qm);

	return 0;
}

static struct of_device_id hisi_hpre_dt_ids[] = {
	{ .compatible = "hisilicon,hip07-hpre-vf"},
	{ },
};

static struct platform_driver hpre_driver = {
	.probe = hpre_probe,
	.remove = hpre_remove,
	.driver = {
		.name = "hisi_plat_hpre",
		.of_match_table = hisi_hpre_dt_ids,
	}
};

module_platform_driver(hpre_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Yuzexi <yuzexi@hisilicon.com>");
MODULE_DESCRIPTION("Driver for HiSilicon HPRE accelerator");
