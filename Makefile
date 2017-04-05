#
# Module Makefile for DM510
#

# Change this if you keep your files elsewhere
ROOT = ..
KERNELDIR = ${ROOT}/linux-3.18.48
PWD = $(shell pwd)


obj-m += dm510_dev.o

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) LDDINC=$(KERNELDIR)/include ARCH=um modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions
