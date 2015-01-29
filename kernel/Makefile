KDIR ?=
currentdir = $(realpath .)

INSTALL_DIR ?= $(currentdir)/bin

obj-m := fsl_tti.o

PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	rm *.o *.ko  *.order *.symvers *.mod.c

install:
	mkdir -p ${INSTALL_DIR}/usr/driver/rf-util;
	cp *.ko ${INSTALL_DIR}/usr/driver/rf-util/
