KDIR ?=

obj-m := fsl_tti.o

PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	rm *.o *.ko  *.order *.symvers *.mod.c
