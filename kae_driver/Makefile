obj-m += hisi_plat_qm.o
hisi_plat_qm-objs = plat_qm.o
obj-m += hisi_plat_hpre.o
hisi_plat_hpre-objs = plat_hpre_main.o
obj-m += hisi_plat_sec.o
hisi_plat_sec-objs = plat_sec_main.o

ccflags-y += -I$(srctree)/drivers/crypto/hisilicon/
KERNEL_DIR ?= /usr/src/kernels/$(shell uname -r)
PWD:=$(shell pwd)

modules:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules
modules_install:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean