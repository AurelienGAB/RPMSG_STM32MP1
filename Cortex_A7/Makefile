PATH:=$(PATH):~/Desktop/build-stm32mp1/host/usr/bin

obj-m +=stm32-rpmsg.o

all:
	make ARCH=arm CROSS_COMPILE=~/Desktop/build-stm32mp1/host/usr/bin/arm-buildroot-linux-uclibcgnueabihf- -C\
		~/Desktop/build-stm32mp1/build/linux-5.8.13 M=$(PWD) modules

clean: 
	rm *.o *.ko *.mod.c *.mod *.order *.symvers 
