/*
 * show virtCCA secure memory driver for Linux
 *
 * Copyright (C) 2024 Huawei Technologies Co., Ltd.
 *
 * This driver provide three linux file system in /sys/kernel/tmm
 * to allow you to get the current memory usage.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kprobes.h>
#include <linux/kallsyms.h>
#include <asm/page.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chenzheng <chenzheng71@huawei.com>");
MODULE_DESCRIPTION("Get tmm base memory info module");

#define TMM_MAX_NODE_NUM 8
#define MAX_PAGE_ORDER_NUM 10
#define MATA_DATA_ARRAY_COUNT 3
#define MAX_TMP_FORMAT_SIZE 100
#define ORDERED_PAGE_SIZE (1ULL << 21)

struct node_mem {
    uint32_t nr_free[MAX_PAGE_ORDER_NUM];
    uint64_t node_mem_size;
};

typedef struct node_mem node_mem_s;

struct meta_datas_info {
    uint64_t reserved_block_cnt;
    uint64_t reserved_block_free_cnt;
    uint64_t reserved_block_size;
    uint64_t extend_blocks_cnt;
    uint64_t extend_blocks_free_cnt;
    uint64_t extend_blocks_size;
};

typedef struct meta_datas_info meta_datas_info_s;

struct tmm_memory_info {
    node_mem_s node_mems[TMM_MAX_NODE_NUM];
    meta_datas_info_s td_mems[TMM_MAX_NODE_NUM];
    meta_datas_info_s tec_mems[TMM_MAX_NODE_NUM];
    meta_datas_info_s ttt_mems[TMM_MAX_NODE_NUM];
};

typedef struct tmm_memory_info tmm_memory_info_s;

static char g_format_buffer[PAGE_SIZE];

static const char *meta_data_strs[] = {
    "td",
    "tec",
    "ttt"
};

typedef enum {
    KAE_VF_INFO,
    TMM_INFO_MAX
}tmm_info_option;

typedef unsigned long (*kallsyms_lookup_name_t)(const char *name);
static kallsyms_lookup_name_t fn_kallsyms_lookup_name = NULL;

typedef uint64_t (*tmi_mem_info_show_t)(uint64_t addr);
static tmi_mem_info_show_t tmi_mem_info_show_func = NULL;

typedef bool (*is_virtcca_available_t)(void);
static is_virtcca_available_t is_virtcca_available_func = NULL;

typedef uint64_t (*tmi_tmm_info_show_t)(uint64_t option, uint64_t tmm_info_addr);
static tmi_tmm_info_show_t tmi_tmm_info_show_func = NULL;

static struct kprobe kp_sym_lookup = {
    .symbol_name = "kallsyms_lookup_name",
};

static void get_kallsyms_lookup_name(void)
{
    int ret;

    ret = register_kprobe(&kp_sym_lookup);
    if (ret < 0) {
        pr_err("tmm_driver: register_kprobe failed, returned %d\n", ret);
    }

    fn_kallsyms_lookup_name = (void *)kp_sym_lookup.addr;
    unregister_kprobe(&kp_sym_lookup);
}

static int get_symbol_from_kernel(void)
{
    get_kallsyms_lookup_name();
    if (!fn_kallsyms_lookup_name) {
        pr_err("tmm_driver: cannot get function kallsyms_lookup_name\n");
        return -1;
    }

    tmi_mem_info_show_func = (tmi_mem_info_show_t)fn_kallsyms_lookup_name("tmi_mem_info_show");
    if (!tmi_mem_info_show_func) {
	pr_err("tmm_driver: cannot get function tmi_mem_info_show\n");
    	return -1;
    }

    is_virtcca_available_func = (is_virtcca_available_t)fn_kallsyms_lookup_name("is_virtcca_available");
    if (!is_virtcca_available_func) {
        pr_err("tmm_driver: cannot get function is_virtcca_available\n");
        return -1;
    }

    tmi_tmm_info_show_func = (tmi_tmm_info_show_t)fn_kallsyms_lookup_name("tmi_tmm_info_show");
    if (!tmi_mem_info_show_func) {
        pr_err("tmm_driver: cannot get function tmi_tmm_info_show\n");
        return -1;
    }

    return 0;
}

static ssize_t virtcca_enabled_show(struct kobject *kobj,
                                    struct kobj_attribute *attr,
                                    char *buf)
{
    return sysfs_emit(buf, "%d\n", is_virtcca_available_func());
}

static int get_tmm_memory_info(tmm_memory_info_s *memory_info)
{
    return tmi_mem_info_show_func((uint64_t)memory_info);
}

static int get_tmm_info(tmm_info_option option, tmm_memory_info_s *tmm_info)
{
    return tmi_tmm_info_show_func(option, (uint64_t)tmm_info);
}

static ssize_t kae_vf_nums_show(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                char *buf)
{
    int ret;

    ret = get_tmm_info(KAE_VF_INFO, NULL);
    return sysfs_emit(buf, "%d\n", ret);
}

static uint64_t cal_numa_node_mem_info(tmm_memory_info_s *memory_info,
                                       int numa_id,
                                       uint64_t *cvm_used,
                                       uint64_t *meta_data_used)
{
    int index;
    uint64_t meta_data_size = 0;
    uint64_t mem_free = 0;
    meta_datas_info_s *meta_data_ptr = &memory_info->td_mems[numa_id];

    for (index = 0; index < MATA_DATA_ARRAY_COUNT; index++) {
        meta_data_size += meta_data_ptr->reserved_block_size;
        meta_data_size += meta_data_ptr->extend_blocks_size;
        meta_data_ptr += TMM_MAX_NODE_NUM;
    }

    for (index = 0; index < MAX_PAGE_ORDER_NUM; index++)
        mem_free += ((ORDERED_PAGE_SIZE << index) * memory_info->node_mems[numa_id].nr_free[index]);

    *meta_data_used = meta_data_size;
    *cvm_used = (memory_info->node_mems[numa_id].node_mem_size - mem_free - meta_data_size);

    return mem_free;
}

static void memory_info_format_numa_bit_map(char **buf,
                                            int numa_bit_map)
{
    int buf_size = 0, index = 0;
    int numa_avalible_size = 0;
    int isFirst = 1;
    int temp_bit_map = numa_bit_map;

    while (temp_bit_map != 0) {
        numa_avalible_size += (temp_bit_map & 1);
        temp_bit_map >>= 1;
    }

    buf_size += snprintf(*buf + buf_size, MAX_TMP_FORMAT_SIZE - buf_size,
                         "available: %d numa nodes (", numa_avalible_size);

    temp_bit_map = numa_bit_map;
    for (index = 0; index < TMM_MAX_NODE_NUM; index++, temp_bit_map >>= 1) {
        if ((temp_bit_map & 1) == 0)
            continue;

        if (isFirst != 0) {
            buf_size += snprintf(*buf + buf_size, MAX_TMP_FORMAT_SIZE - buf_size,
                                 "%d", index);
            isFirst = 0;
        } else {
            buf_size += snprintf(*buf + buf_size, MAX_TMP_FORMAT_SIZE - buf_size,
                                 " %d", index);
        }
    }

    buf_size += snprintf(*buf + buf_size, MAX_TMP_FORMAT_SIZE - buf_size, ")");
}

static ssize_t memory_info_format(char **buf,
                                  tmm_memory_info_s *memory_info)
{
    int numa_index;
    int numa_bit_map = 0;
    char format_buffer[MAX_TMP_FORMAT_SIZE];
    char head_buffer[MAX_TMP_FORMAT_SIZE];
    char *head_buffer_ptr = head_buffer;
    uint64_t mem_node_free, cvm_used, meta_data_used;
    ssize_t memory_info_size = 0;

    for (numa_index = 0; numa_index < TMM_MAX_NODE_NUM; numa_index++) {
        if (memory_info->node_mems[numa_index].node_mem_size == 0)
            continue;

        numa_bit_map |= (1 << numa_index);
        mem_node_free = cal_numa_node_mem_info(memory_info, numa_index, &cvm_used, &meta_data_used);
        snprintf(head_buffer, MAX_TMP_FORMAT_SIZE, "numa node %d", numa_index);

        snprintf(format_buffer, MAX_TMP_FORMAT_SIZE, "%s %s", head_buffer, "size:");
        memory_info_size += snprintf(g_format_buffer + memory_info_size, PAGE_SIZE - memory_info_size,
                                     "%-35s%lluMi\n", format_buffer,
                                     memory_info->node_mems[numa_index].node_mem_size >> 20);
        snprintf(format_buffer, MAX_TMP_FORMAT_SIZE, "%s %s", head_buffer, "free:");
        memory_info_size += snprintf(g_format_buffer + memory_info_size, PAGE_SIZE - memory_info_size,
                                     "%-35s%lluMi\n", format_buffer, mem_node_free >> 20);
        snprintf(format_buffer, MAX_TMP_FORMAT_SIZE, "%s %s", head_buffer, "cvm used:");
        memory_info_size += snprintf(g_format_buffer + memory_info_size, PAGE_SIZE - memory_info_size,
                                     "%-35s%lluMi\n", format_buffer, cvm_used >> 20);
        snprintf(format_buffer, MAX_TMP_FORMAT_SIZE, "%s %s", head_buffer, "meta_data used:");
        memory_info_size += snprintf(g_format_buffer + memory_info_size, PAGE_SIZE - memory_info_size,
                                     "%-35s%lluMi\n", format_buffer, meta_data_used >> 20);
    }

    memset((void *)head_buffer, 0, sizeof(head_buffer));
    memory_info_format_numa_bit_map(&head_buffer_ptr, numa_bit_map);

    memory_info_size = snprintf(*buf, PAGE_SIZE, "%s\n%s", head_buffer, g_format_buffer);

    return memory_info_size;
}

static ssize_t memory_info_show(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                char *buf)
{
    int ret;
    ssize_t memory_info_size;
    tmm_memory_info_s *memory_info = kzalloc(sizeof(tmm_memory_info_s), GFP_KERNEL_ACCOUNT);
    if (!memory_info) {
        pr_err("tmm_driver: memory_info alloc failed\n");
        return -1;
    }

    ret = get_tmm_memory_info(memory_info);
    if (!ret) {
        memory_info_size = memory_info_format(&buf, memory_info);
    } else {
        memory_info_size = -1;
        pr_err("tmm_drivr: unable to get memory_info\n");
    }

    kfree(memory_info);
    return memory_info_size;
}

static ssize_t meta_datas_format(char **buf,
                                tmm_memory_info_s *memory_info,
                                int numa_index,
                                ssize_t offset)
{
    int index;
    char format_buffer[MAX_TMP_FORMAT_SIZE];
    char head_buffer[MAX_TMP_FORMAT_SIZE];
    uint64_t meta_data_free, meta_data_total;
    ssize_t meta_data_info_size = offset;
    meta_datas_info_s *tx_ptr = &memory_info->td_mems[numa_index];

    if (memory_info->node_mems[numa_index].node_mem_size == 0)
        return meta_data_info_size;

    for (index = 0; index < MATA_DATA_ARRAY_COUNT; index++, tx_ptr += TMM_MAX_NODE_NUM) {
        meta_data_total = tx_ptr->reserved_block_cnt + tx_ptr->extend_blocks_cnt;
        meta_data_free = tx_ptr->reserved_block_free_cnt + tx_ptr->extend_blocks_free_cnt;
        snprintf(head_buffer, MAX_TMP_FORMAT_SIZE, "numa node %d %s", numa_index, meta_data_strs[index]);

        snprintf(format_buffer, MAX_TMP_FORMAT_SIZE, "%s %s", head_buffer, "meta_data cnt:");
        meta_data_info_size += snprintf(*buf + meta_data_info_size, PAGE_SIZE - meta_data_info_size,
                                        "%-35s%llu\n", format_buffer, meta_data_total);
        snprintf(format_buffer, MAX_TMP_FORMAT_SIZE, "%s %s", head_buffer, "meta_data free cnt:");
        meta_data_info_size += snprintf(*buf + meta_data_info_size, PAGE_SIZE - meta_data_info_size,
                                        "%-35s%llu\n", format_buffer, meta_data_free);
    }

    return meta_data_info_size;
}

static ssize_t slab_info_format(char **buf,
                                tmm_memory_info_s *memory_info)
{
    int numa_index;
    ssize_t slab_info_size = 0;

    for (numa_index = 0; numa_index < TMM_MAX_NODE_NUM; numa_index++)
        slab_info_size = meta_datas_format(buf, memory_info, numa_index, slab_info_size);

    return slab_info_size;
}

static ssize_t slab_info_show(struct kobject *kobj,
                              struct kobj_attribute *attr,
                              char *buf)
{
    int ret;
    ssize_t slab_info_size;
    tmm_memory_info_s *memory_info = kzalloc(sizeof(tmm_memory_info_s), GFP_KERNEL_ACCOUNT);
    if (!memory_info) {
        pr_err("tmm_driver: memory_info alloc failed\n");
        return -1;
    }

    ret = get_tmm_memory_info(memory_info);
    if (!ret) {
        slab_info_size = slab_info_format(&buf, memory_info);
    } else {
        slab_info_size = -1;
        pr_err("tmm_driver: unable to get memory_info\n");
    }

    kfree(memory_info);
    return slab_info_size;
}

static void format_node_order_buffer(char **buf,
                                     tmm_memory_info_s *memory_info,
                                     int node_index)
{
    int numa_index;
    ssize_t orderx_format_size = 0;

    orderx_format_size += snprintf(*buf + orderx_format_size, MAX_TMP_FORMAT_SIZE - orderx_format_size,
                                   "%s%d:", "order", node_index);

    for (numa_index = 0; numa_index < TMM_MAX_NODE_NUM; numa_index++) {
        if (memory_info->node_mems[numa_index].node_mem_size == 0)
            continue;

        orderx_format_size += snprintf(*buf + orderx_format_size, MAX_TMP_FORMAT_SIZE - orderx_format_size,
                                       "%4u", memory_info->node_mems[numa_index].nr_free[node_index]);
    }
}

static ssize_t buddy_info_format(char **buf,
                                 tmm_memory_info_s *memory_info)
{
    int numa_index, node_index;
    char node_format[MAX_TMP_FORMAT_SIZE];
    char orderx_format[MAX_TMP_FORMAT_SIZE];
    char *orderx_format_ptr = orderx_format;
    ssize_t buddy_info_size = 0, node_format_size = 0;

    buddy_info_size += snprintf(*buf + buddy_info_size, PAGE_SIZE - buddy_info_size,
                                "numa node free page number per order:\n");
    node_format_size += snprintf(node_format + node_format_size, PAGE_SIZE - node_format_size, "%-7s", "node");

    for (numa_index = 0; numa_index < TMM_MAX_NODE_NUM; numa_index++) {
        if (memory_info->node_mems[numa_index].node_mem_size == 0)
            continue;

        node_format_size += snprintf(node_format + node_format_size, MAX_TMP_FORMAT_SIZE - node_format_size,
                                     "%4d", numa_index);
    }

    buddy_info_size += snprintf(*buf + buddy_info_size, PAGE_SIZE - buddy_info_size, "%s\n", node_format);

    for (node_index = 0; node_index < MAX_PAGE_ORDER_NUM; node_index++) {
        format_node_order_buffer(&orderx_format_ptr, memory_info, node_index);
        buddy_info_size += snprintf(*buf + buddy_info_size, PAGE_SIZE - buddy_info_size, "%s\n", orderx_format);
    }

    return buddy_info_size;
}

static ssize_t buddy_info_show(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               char *buf)
{
    int ret;
    ssize_t buddy_info_size;
    tmm_memory_info_s *memory_info = kzalloc(sizeof(tmm_memory_info_s), GFP_KERNEL_ACCOUNT);

    if (!memory_info) {
        pr_err("tmm_driver: memory_info alloc failed\n");
        return -1;
    }

    ret = get_tmm_memory_info(memory_info);
    if (!ret) {
        buddy_info_size = buddy_info_format(&buf, memory_info);
    } else {
        buddy_info_size = -1;
        pr_err("tmm_driver: unable to get memory_info\n");
    }

    kfree(memory_info);
    return buddy_info_size;
}

static struct kobj_attribute virtcca_enabled_attr = __ATTR_RO(virtcca_enabled);
static struct kobj_attribute memory_info_attr = __ATTR_RO(memory_info);
static struct kobj_attribute slab_info_attr = __ATTR_RO(slab_info);
static struct kobj_attribute buddy_info_attr = __ATTR_RO(buddy_info);
static struct kobj_attribute kae_vf_nums_attr = __ATTR_RO(kae_vf_nums);
static struct kobject *tmm_kobj;

static struct attribute *tmm_attrs[] = {
    &virtcca_enabled_attr.attr,
    &memory_info_attr.attr,
    &slab_info_attr.attr,
    &buddy_info_attr.attr,
    &kae_vf_nums_attr.attr,
    NULL,
};

static int __init tmm_driver_init(void)
{
    int rc, i;

    tmm_kobj = kobject_create_and_add("tmm", kernel_kobj);
    if (tmm_kobj == NULL) {
        pr_err("tmm_driver: create tmm kobject failed\n");
        return -1;
    }

    for (i = 0; tmm_attrs[i] != NULL; i++) {
        rc = sysfs_create_file(tmm_kobj, tmm_attrs[i]);
        if (rc) {
            pr_err("tmm_driver: unable to create sysfs file (%d)\n", rc);
            goto err;
        }
    }

    rc = get_symbol_from_kernel();
    if (rc)
        goto err;

    return 0;

err:
    for (--i; i >= 0; i--) {
        sysfs_remove_file(tmm_kobj, tmm_attrs[i]);
    }

    kobject_put(tmm_kobj);

    return rc;
}

static void __exit tmm_driver_exit(void)
{
    int i;

    for (i = 0; tmm_attrs[i] != NULL; i++) {
        sysfs_remove_file(tmm_kobj, tmm_attrs[i]);
    }

    kobject_put(tmm_kobj);
    printk(KERN_INFO "tmm_driver_exit!\n");
}

module_init(tmm_driver_init);
module_exit(tmm_driver_exit);
