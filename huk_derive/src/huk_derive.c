/*
 * show virtCCA secure memory driver for Linux
 *
 * Copyright (C) 2024 Huawei Technologies Co., Ltd.
 *
 * This driver provide huk derive key.
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/arm-smccc.h>

#define HUK_DERIVE_KEY_LEN 32
#define USER_PARAM_LEN 64

struct huk_derive_params
{
    uint8_t user_param[USER_PARAM_LEN];
    uint32_t user_param_len;
    uint8_t derived_key[HUK_DERIVE_KEY_LEN];
};

#define TSI_HUK_DEV "tsi_huk"
#define HUK_IOC_MAGIC 'd'
#define HUK_IOCTL_DERIVE_KEY _IOWR(HUK_IOC_MAGIC, 0, struct huk_derive_params)

#define ERR(fmt, args...) (void)pr_err("[%s][%s] " fmt "\n", TSI_HUK_DEV, __func__, ##args)
#define INFO(fmt, args...) (void)pr_info("[%s][%s] " fmt "\n", TSI_HUK_DEV, __func__, ##args)

#define SMC_TSI_CALL_BASE 0xC4000000
#define SMC_TSI_FID(_x) (SMC_TSI_CALL_BASE + (_x))
#define SCM_TSI_HUK_DERIVE_KEY SMC_TSI_FID(0x19B)

static struct class *g_driver_class;
static dev_t g_devno;
static struct cdev g_cdev;

static int tsi_derive_huk(void *user_buf, uint32_t len, void *key_buf)
{
    struct arm_smccc_res res = {0};
    arm_smccc_1_1_smc(SCM_TSI_HUK_DERIVE_KEY, virt_to_phys(user_buf), len, virt_to_phys(key_buf), &res);

    return res.a0;
}

static int cmd_get_derived_key(void __user *argp)
{
    struct huk_derive_params huk_params = {0};
    int ret = 0;
    uint8_t *user_param = NULL;
    uint8_t *derived_key = NULL;

    if (!argp) {
        ERR("args is NULL\n");
        return -EINVAL;
    }

    ret = copy_from_user(&huk_params, argp, sizeof(struct huk_derive_params));
    if (ret) {
        ERR("copy huk params failed");
        return ret;
    }

    if (huk_params.user_param_len != 0) {
        user_param = kzalloc(huk_params.user_param_len, GFP_KERNEL);
        if (!user_param) {
            ERR("malloc memory failed");
            return -ENOMEM;
        }
        (void)memcpy(user_param, huk_params.user_param, huk_params.user_param_len);
    }

    derived_key = kzalloc(HUK_DERIVE_KEY_LEN, GFP_KERNEL);
    if (!derived_key) {
        ERR("malloc memory failed");
        ret = -ENOMEM;
        goto free_umem;
    }

    ret = tsi_derive_huk(user_param, huk_params.user_param_len, derived_key);
    if (ret != SMCCC_RET_SUCCESS) {
        ERR("got tsi huk derive key failed, ret: %d\n", ret);
        goto free_kmem;
    }

    (void)memset(huk_params.derived_key, 0, HUK_DERIVE_KEY_LEN);
    (void)memcpy(huk_params.derived_key, derived_key, HUK_DERIVE_KEY_LEN);
    ret = copy_to_user(argp, &huk_params, sizeof(struct huk_derive_params));
    if (ret) {
        ERR("copy result failed");
    }

free_kmem:
    kfree(derived_key);
free_umem:
    kfree(user_param);
    return ret;
}

static long tsi_device_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    void *argp = (void __user *)(uintptr_t)arg;

    switch (cmd) {
    case HUK_IOCTL_DERIVE_KEY:
        ret = cmd_get_derived_key(argp);
        break;
    default:
        ERR("invalid command %d", cmd);
        ret = -ENOTTY;
    }

    return ret;
}

static struct file_operations g_huk_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = tsi_device_ioctl,
};

static int __init huk_init(void)
{
    int ret;
    struct device *class_dev = NULL;

    g_driver_class = class_create(THIS_MODULE, TSI_HUK_DEV);
    if (IS_ERR_OR_NULL(g_driver_class)) {
        ERR("class create failed");
        return -ENOMEM;
    }

    ret = alloc_chrdev_region(&g_devno, 0, 1, TSI_HUK_DEV);
    if (ret < 0) {
        ERR("alloc_chrdev_region failed!");
        goto free_class;
    }
    INFO("MAJOR is %d, MINOR is %d", MAJOR(g_devno), MINOR(g_devno));
    class_dev = device_create(g_driver_class, NULL, g_devno, NULL, TSI_HUK_DEV);
    if (IS_ERR_OR_NULL(class_dev)) {
        ERR("device create failed");
        ret = -ENOMEM;
        goto unreg_chrdev;
    }

    cdev_init(&g_cdev, &g_huk_fops);
    ret = cdev_add(&g_cdev, g_devno, 1);
    if (ret < 0) {
        ERR("cdev_add failed");
        goto free_device;
    }
    INFO("module init success");
    return 0;

free_device:
    device_destroy(g_driver_class, g_devno);
unreg_chrdev:
    unregister_chrdev_region(g_devno, 1);
free_class:
    class_destroy(g_driver_class);
    return ret;
}

static void __exit huk_exit(void)
{
    device_destroy(g_driver_class, g_devno);
    cdev_del(&g_cdev);
    unregister_chrdev_region(g_devno, 1);
    class_destroy(g_driver_class);
    INFO("module exited");
    return;
}

module_init(huk_init);
module_exit(huk_exit);
MODULE_LICENSE("GPL");