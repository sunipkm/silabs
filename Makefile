# obj-$(SI446X_DEV) += si446x.o
obj-m += si446x.o
KERNELPATH=~/rpi-linux

all:
	make -C $(KERNELPATH) M=$(PWD) modules

clean:
	make -C $(KERNELPATH) M=$(PWD) clean