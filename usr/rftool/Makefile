CFLAGS  += -c -msoft-float
LDFLAGS += -msoft-float -lm

INCLUDES += -I${INTERFACE_DIR}

SRCS := $(wildcard *.c)
OBJS := $(patsubst %.c, %.o, ${SRCS})

all: rftool

rftool: ${OBJS}
	${CC} ${LDFLAGS} ${OBJS} -o $@ ${LIB_DIR}/rflib.a

%.o: %.c
	${CC} ${CFLAGS} ${INCLUDES}  $< -o $@

clean:
	rm -rf *.o rftool

install:
	cp rftool ${INSTALL_DIR}/



