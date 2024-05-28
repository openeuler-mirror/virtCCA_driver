.PHONY: clean tmm seal

all: tmm seal

tmm:
	make -C tmm_driver/src

seal:
	make -C sealing_key/src

clean:
	make clean -C tmm_driver/src
	make clean -C sealing_key/src