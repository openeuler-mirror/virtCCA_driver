/*
 * get cvm sealing key driver for Linux
 *
 * Copyright (C) 2024 Huawei Technologies Co., Ltd.
 *
 * This driver provide huk derive key.
 */

#include <linux/version.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/arm-smccc.h>

#define SEALING_KEY_LEN 32
#define SEALING_PARAM_LEN 64

struct sealing_key_params {
    uint32_t alg;
    uint8_t user_param[SEALING_PARAM_LEN];
    uint32_t user_param_len;
    uint8_t sealing_key[SEALING_KEY_LEN];
};

typedef enum {
    SEALING_HMAC_SHA256
} SEALING_KEY_ALG;

#define SEALING_KEY_DEV "sealingkey"
#define SEAL_KEY_IOC_MAGIC 'd'
#define IOCTL_SEALING_KEY _IOWR(SEAL_KEY_IOC_MAGIC, 0, struct sealing_key_params)

#define ERR(fmt, args...) (void)pr_err("[%s][%s] " fmt "\n", SEALING_KEY_DEV, __func__, ##args)
#define INFO(fmt, args...) (void)pr_info("[%s][%s] " fmt "\n", SEALING_KEY_DEV, __func__, ##args)

#define SMC_TSI_CALL_BASE 0xC4000000
#define SMC_TSI_FID(_x) (SMC_TSI_CALL_BASE + (_x))
#define SMC_TSI_HUK_DERIVE_KEY SMC_TSI_FID(0x19B)

static struct class *g_driver_class;
static dev_t g_devno;
static struct cdev g_cdev;

static int smc_sealing_key(uint32_t alg, uint8_t *user_param, uint32_t user_param_len, uint8_t *key_buf)
{
    struct arm_smccc_res res = {0};
    arm_smccc_1_1_smc(SMC_TSI_HUK_DERIVE_KEY, alg, virt_to_phys(user_param), user_param_len,
                      virt_to_phys(key_buf), &res);

    return res.a0;
}

static int check_malloc_params(struct sealing_key_params *seal_params, uint8_t **user_param, uint8_t **sealing_key)
{
    if (seal_params->alg != SEALING_HMAC_SHA256) {
        ERR("not support alg %u\n", seal_params->alg);
        return -EINVAL;
    }
    if (seal_params->user_param_len != 0 && seal_params->user_param_len != SEALING_PARAM_LEN) {
        ERR("invalid user_param len %u", seal_params->user_param_len);
        return -EINVAL;
    }

    *sealing_key = kzalloc(SEALING_KEY_LEN, GFP_KERNEL);
    if (*sealing_key == NULL) {
        ERR("malloc memory failed");
        return -ENOMEM;
    }

    if (seal_params->user_param_len != 0) {
        *user_param = kzalloc(seal_params->user_param_len, GFP_KERNEL);
        if (*user_param == NULL) {
            ERR("malloc memory failed");
            goto err;
        }
        (void)memcpy(*user_param, seal_params->user_param, seal_params->user_param_len);
    }
    return 0;

err:
    kfree(*sealing_key);
    *sealing_key = NULL;
    return -ENOMEM;
}

static int cmd_get_sealing_key(void __user *argp)
{
    struct sealing_key_params seal_params = {0};
    int ret = 0;
    uint8_t *user_param = NULL;
    uint8_t *sealing_key = NULL;

    if (!argp) {
        ERR("args is NULL\n");
        return -EINVAL;
    }

    ret = copy_from_user(&seal_params, argp, sizeof(struct sealing_key_params));
    if (ret) {
        ERR("copy huk params failed");
        return ret;
    }

    ret = check_malloc_params(&seal_params, &user_param, &sealing_key);
    if (ret) {
        return ret;
    }
 
    ret = smc_sealing_key(seal_params.alg, user_param, seal_params.user_param_len, sealing_key);
    if (ret != SMCCC_RET_SUCCESS) {
        ERR("got sealing key by smc failed, ret: %d\n", ret);
        goto free_kmem;
    }

    (void)memcpy(seal_params.sealing_key, sealing_key, SEALING_KEY_LEN);
    (void)memset(sealing_key, 0, SEALING_KEY_LEN);
    ret = copy_to_user(argp, &seal_params, sizeof(struct sealing_key_params));
    if (ret) {
        ERR("copy result failed");
    } else {
        INFO("sealing key success");
    }
    (void)memset(&seal_params, 0, sizeof(seal_params));

free_kmem:
    kfree(sealing_key);
    if (user_param) {
        kfree(user_param);
    }
    return ret;
}

static long seal_device_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    void *argp = (void __user *)(uintptr_t)arg;

    switch (cmd) {
    case IOCTL_SEALING_KEY:
        ret = cmd_get_sealing_key(argp);
        break;
    default:
        ERR("invalid command %u", cmd);
        ret = -ENOTTY;
    }

    return ret;
}

static struct file_operations g_seal_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = seal_device_ioctl,
};

static int __init seal_init(void)
{
    int ret;
    struct device *class_dev = NULL;

#if (KERNEL_VERSION(6, 4, 0) <= LINUX_VERSION_CODE)
    g_driver_class = class_create(SEALING_KEY_DEV);
#else
    g_driver_class = class_create(THIS_MODULE, SEALING_KEY_DEV);
#endif
    if (IS_ERR_OR_NULL(g_driver_class)) {
        ERR("class create failed");
        return -ENOMEM;
    }

    ret = alloc_chrdev_region(&g_devno, 0, 1, SEALING_KEY_DEV);
    if (ret < 0) {
        ERR("alloc_chrdev_region failed!");
        goto free_class;
    }
    INFO("MAJOR is %d, MINOR is %d", MAJOR(g_devno), MINOR(g_devno));
    class_dev = device_create(g_driver_class, NULL, g_devno, NULL, SEALING_KEY_DEV);
    if (IS_ERR_OR_NULL(class_dev)) {
        ERR("device create failed");
        ret = -ENOMEM;
        goto unreg_chrdev;
    }

    cdev_init(&g_cdev, &g_seal_fops);
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

static void __exit seal_exit(void)
{
    device_destroy(g_driver_class, g_devno);
    cdev_del(&g_cdev);
    unregister_chrdev_region(g_devno, 1);
    class_destroy(g_driver_class);
    INFO("module exited");
    return;
}

module_init(seal_init);
module_exit(seal_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chenzheng <chenzheng71@huawei.com>");
MODULE_DESCRIPTION("Get sealing key from tmm module");