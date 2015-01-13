CFLAGS  += -c -msoft-float

INCLUDES += -I${INTERFACE_DIR}

# IPC and kernel includes are for using user space
# memory manager.

INCLUDES += -I${IPC_DIR}/ipc/include/
INCLUDES += -I${IPC_DIR}/fsl_shm/include/
INCLUDES += -I${KERNEL_DIR}/drivers/misc/

SRCS := $(wildcard *.c)
OBJS := $(patsubst %.c, %.o, ${SRCS})

all: rflib

rflib: prep ${OBJS}
	mkdir -p ${LIB_DIR};
	${AR} cr rflib.a ${OBJS}
	cp rflib.a ${LIB_DIR}/

%.o: %.c
	${CC} ${CFLAGS} ${INCLUDES}  $< -o $@

prep:
	@if [ "${KERNEL_DIR}" != "" ] ;						\
	then 									\
		cp ${KERNEL_DIR}/include/linux/rfdev_ioctl.h ${INTERFACE_DIR}/;	\
	fi

clean:
	rm -rf *.o rflib.a

install:
	cp rflib.a ${LIB_DIR}/



