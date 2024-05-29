.PHONY: clean tmm seal

KERNEL_DIR ?= /usr/src/kernels/$(shell uname -r)

all: tmm seal

tmm:
	make KERNEL_DIR=$(KERNEL_DIR) -C tmm_driver/src

seal:
	make -C sealing_key/src

clean:
	make clean -C tmm_driver/src
	make clean -C sealing_key/src
