CC = /opt/fsl-networking/QorIQ-SDK-V1.6/sysroots/x86_64-fslsdk-linux/usr/bin/powerpc-fsl-linux-gnuspe/powerpc-fsl-linux-gnuspe-gcc
AR = /opt/fsl-networking/QorIQ-SDK-V1.6/sysroots/x86_64-fslsdk-linux/usr/bin/powerpc-fsl-linux-gnuspe/powerpc-fsl-linux-gnuspe-ar


INSTALL_DIR ?= ${PWD}/bin
LIB_DIR ?= ${PWD}/bin
INTERFACE_DIR := ${PWD}/interface
INTERFACE_DIR := ${PWD}/interface
CONFIG_DIR ?=
INCLUDES += -I${INTERFACE_DIR}/
INCLUDES += -I${PWD}/rflib/
INCLUDES += -I${PWD}/rftool/
#CFLAGS += -DDEBUG
CFLAGS += -Wall

export CC  INSTALL_DIR LIB_DIR CONFIG_DIR
export INTERFACE_DIR
export INCLUDES CFLAGS
export IPC_DIR

DIRS := rflib rftool

all: ${DIRS}
	$(foreach b, $(DIRS), ${MAKE} -C ${b}  all;)

clean: ${DIRS}
	$(foreach b, $(DIRS), ${MAKE} -C ${b}  clean;)

install: ${DIRS}
	mkdir -p ${INSTALL_DIR};
	@if [ "${CONFIG_DIR}" != "" ] ;			\
	then 						\
		cp -r ${PWD}/config/* ${CONFIG_DIR}/;	\
	fi
	$(foreach b, $(DIRS), ${MAKE} -C ${b}  install;)
