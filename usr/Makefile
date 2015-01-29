CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar

currentdir = $(realpath .)

INSTALL_DIR ?= $(currentdir)/
LIB_DIR ?= lib
BIN_DIR ?= bin
INTERFACE_DIR := $(currentdir)/interface
#INSTALL_DIR ?= ${PWD}/bin
#LIB_DIR ?= ${PWD}/bin
#INTERFACE_DIR := ${PWD}/interface
CONFIG_DIR ?=
INCLUDES += -I${INTERFACE_DIR}/
#INCLUDES += -I${PWD}/rflib/
#INCLUDES += -I${PWD}/rftool/
INCLUDES += -I $(currentdir)/rflib
INCLUDES += -I $(currentdir)/rftool
#CFLAGS += -DDEBUG
CFLAGS += -Wall

export CC  INSTALL_DIR LIB_DIR CONFIG_DIR BIN_DIR
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
