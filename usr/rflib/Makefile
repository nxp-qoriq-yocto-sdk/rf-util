CFLAGS  += -c -msoft-float

INCLUDES += -I${INTERFACE_DIR}

# IPC and kernel includes are for using user space
# memory manager.

INCLUDES += -I${IPC_DIR}/ipc/include/
INCLUDES += -I${IPC_DIR}/fsl_shm/include/
INCLUDES += -I${KERNEL_DIR}/drivers/misc/

SRCS := $(wildcard *.c)
OBJS := $(patsubst %.c, %.o, ${SRCS})

all: librflib

librflib: prep ${OBJS}
	mkdir -p ${INSTALL_DIR}/usr/${LIB_DIR};
	${AR} cr librflib.a ${OBJS}
	cp librflib.a ${INSTALL_DIR}/usr/${LIB_DIR}/

%.o: %.c
	${CC} ${CFLAGS} ${INCLUDES}  $< -o $@

prep:
	@if [ "${KERNEL_DIR}" != "" ] ;						\
	then 									\
		cp ${KERNEL_DIR}/include/linux/rfdev_ioctl.h ${INTERFACE_DIR}/;	\
	fi

clean:
	rm -rf *.o librflib.a ${INSTALL_DIR}/usr

install:
	mkdir -p ${INSTALL_DIR}/usr/${LIB_DIR};
	cp librflib.a ${INSTALL_DIR}/usr/${LIB_DIR}/



