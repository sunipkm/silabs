# obj-$(SI446X_DEV) += si446x.o
obj-m += si446x.o
KERNELPATH=~/rpi-linux

ARCH=$(shell uname -m)
CROSS_COMPILE=CROSS_COMPILE=arm-linux-gnueabihf-
ARCH_NAME=ARCH=arm
ifeq ($(ARCH), arm)
	ARCH_NAME=
	CROSS_COMPILE=
endif
ifeq ($(ARCH), armv7l)
	ARCH_NAME=
	CROSS_COMPILE=
endif

all:
	$(ARCH_NAME) $(CROSS_COMPILE) make -C $(KERNELPATH) M=$(PWD) modules

clean:
	$(ARCH_NAME) $(CROSS_COMPILE) make -C $(KERNELPATH) M=$(PWD) clean

dtb:
	dtc -I dts -O dtb -o si446x-sp0.dtbo si446x-spi0.dts