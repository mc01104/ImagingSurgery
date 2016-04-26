PROC=proc
CC=g++
CP=cp
AR=ar
MAKE=make
ECHO=@echo

INCLUDE_DIR=../../include
LIB_DIR=../../lib
SRLIB_LIB=

TARGET=common.a
OBJS=utils.o

all: ${TARGET}

common.a:${OBJS}
	${AR} rcv $@ ${OBJS}
	ranlib $@

deploy:
	${CP} *.a ${LIB_DIR}
	${CP} *.h ${INCLUDE_DIR}

clean:
	rm -f ${OBJS}
	rm -f *.a

new:
	${MAKE} clean
	${MAKE} all

