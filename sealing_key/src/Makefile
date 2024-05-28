obj-m += sealing_key.o
# Otherwise we were called directly from the command
# line; invoke the kernel build system.
KERNEL_DIR=/usr/src/kernels/$(shell uname -r)
PWD:=$(shell pwd)

modules:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules
modules_install:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean