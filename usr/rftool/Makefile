CFLAGS  += -c -msoft-float
LDFLAGS += -msoft-float -lm

INCLUDES += -I${INTERFACE_DIR}

SRCS := $(wildcard *.c)
OBJS := $(patsubst %.c, %.o, ${SRCS})

all: rftool

rftool: ${OBJS}
	${CC} ${LDFLAGS} ${OBJS} -o $@ ${INSTALL_DIR}/usr/${LIB_DIR}/librflib.a

%.o: %.c
	${CC} ${CFLAGS} ${INCLUDES}  $< -o $@

clean:
	rm -rf *.o rftool

install:
	mkdir -p ${INSTALL_DIR}/usr/${BIN_DIR};
	cp rftool ${INSTALL_DIR}/usr/${BIN_DIR}/



