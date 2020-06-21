TARGET = pl360
ifndef KERNELDIR
	KERNELDIR  := /lib/modules/$(shell uname -r)/build
endif

obj-m += $(TARGET).o
pl360-objs := modpl360.o pl360_ops.o pl360_hw.o

all:
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) clean

install:
	modprobe ieee802154_socket
	modprobe mac802154
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) modules_install
	/sbin/depmod -A

dts:
	dtc -@ -I dts -O dtb -o $(TARGET).dtbo $(TARGET)-overlay.dts
	cp $(TARGET).dtbo /boot/overlays/
