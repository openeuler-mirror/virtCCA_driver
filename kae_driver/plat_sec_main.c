
/*
 * SPDX-License-Identifier: GPL-2.0
 * Copyright (c) Huawei Technologies Co., Ltd. 2024-2024. All rights reserved.
 * Kunpeng hardware accelerator sec module init.
 */

#include "plat_qm.h"

#define SEC_SQE_SIZE			128

struct plat_sec {
	struct hisi_plat_qm qm;
};


static int sec_qm_init(struct hisi_plat_qm *qm, struct platform_device *pdev)
{
	qm->pdev = pdev;
	qm->sqe_size = SEC_SQE_SIZE;
	qm->algs = "sec\ncipher\ndigest\naead\n";

	return platform_init_qm(qm);
}

static void sec_qm_uninit(struct hisi_plat_qm *qm)
{
	platform_uninit_qm(qm);
}

static int sec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hisi_plat_qm *qm;
	struct plat_sec *sec;
	int ret;

	dev_info(dev, "SEC init ...\n");

	sec = devm_kzalloc(&pdev->dev, sizeof(*sec), GFP_KERNEL);
	if (!sec) {
		dev_err(dev, "Fail to alloc sec!\n");
		return -ENOMEM;
	}

	qm = &sec->qm;
	ret = sec_qm_init(qm, pdev);
	if (ret) {
		dev_err(dev, "Fail to init sec qm (%d)!\n", ret);
		return ret;
	}

	ret = platform_start_qm(qm);
	if (ret) {
		dev_err(dev, "Fail to start sec qm!\n");
		goto err_qm_uninit;
	}

	ret = platform_register_uacce(qm);
	if (ret) {
		dev_err(dev, "Fail to register uacce (%d)!\n", ret);
		goto err_qm_stop;
	}

	return 0;
err_qm_stop:
	platform_stop_qm(qm, QM_NORMAL);
err_qm_uninit:
	sec_qm_uninit(qm);

	return ret;
}

static int sec_remove(struct platform_device *pdev)
{
	struct hisi_plat_qm *qm = platform_get_drvdata(pdev);

	(void)platform_stop_qm(qm, QM_NORMAL);
	sec_qm_uninit(qm);

	return 0;
}

static struct of_device_id hisi_sec_dt_ids[] = {
        { .compatible = "hisilicon,hip07-sec-vf"},
        { /* sentinel */ },
};

static struct acpi_device_id hisi_sec_acpi_match[] = {
    {"SEC07", 0},
    { },
};

static struct platform_driver sec_driver = {
	.probe = sec_probe,
	.remove = sec_remove,
	.driver = {
		.name = "hisi_plat_sec",
		.of_match_table = hisi_sec_dt_ids,
		.acpi_match_table = hisi_sec_acpi_match,
	}
};

module_platform_driver(sec_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Leisen <leisen1@huawei.com>");
MODULE_DESCRIPTION("HiSilicon SEC accelerator platform driver");