.PHONY: clean tmm seal

KERNEL_DIR ?= /usr/src/kernels/$(shell uname -r)

all: tmm seal kae

tmm:
	make KERNEL_DIR=$(KERNEL_DIR) -C tmm_driver/src

seal:
	make KERNEL_DIR=$(KERNEL_DIR) -C sealing_key/src

kae:
	make KERNEL_DIR=$(KERNEL_DIR) -C kae_driver

clean:
	make clean KERNEL_DIR=$(KERNEL_DIR) -C tmm_driver/src
	make clean KERNEL_DIR=$(KERNEL_DIR) -C sealing_key/src
	make clean KERNEL_DIR=$(KERNEL_DIR) -C kae_driver
