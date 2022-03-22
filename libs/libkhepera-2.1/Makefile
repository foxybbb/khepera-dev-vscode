#--------------------------------------------------------------------
# Makefile - Khepera Library
#
# Revision 2.0  2015/01/28 10:51:54  jtharin
#
#---------------------------------------------------------------------

SRCS		= $(wildcard *.c)
OBJS		= $(patsubst %.c,%.o,${SRCS})

BUILD		= ../build-${TARGET_SYSTEM}


export TOOL_DIR=/opt/poky/1.8


#### POKY exports
export SDKTARGETSYSROOT=${TOOL_DIR}/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi
export PATH:=${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr/bin:${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi:${PATH}

export CCACHE_PATH:=${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr/bin:${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi:${CCACHE_PATH}
export PKG_CONFIG_SYSROOT_DIR=${SDKTARGETSYSROOT}
export PKG_CONFIG_PATH=${SDKTARGETSYSROOT}/usr/lib/pkgconfig
export CONFIG_SITE=${TOOL_DIR}/site-config-cortexa8hf-vfp-neon-poky-linux-gnueabi
export OECORE_NATIVE_SYSROOT=${TOOL_DIR}/sysroots/i686-pokysdk-linux
export OECORE_TARGET_SYSROOT=${SDKTARGETSYSROOT}
export OECORE_ACLOCAL_OPTS=-I ${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr/share/aclocal
export PYTHONHOME=${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr
export CC=arm-poky-linux-gnueabi-gcc  -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 --sysroot=${SDKTARGETSYSROOT}
export CXX=arm-poky-linux-gnueabi-g++  -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 --sysroot=${SDKTARGETSYSROOT}
export CPP=arm-poky-linux-gnueabi-gcc -E  -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 --sysroot=${SDKTARGETSYSROOT}
export AS=arm-poky-linux-gnueabi-as
export LD=arm-poky-linux-gnueabi-ld  --sysroot=${SDKTARGETSYSROOT}
export GDB=arm-poky-linux-gnueabi-gdb
export STRIP=arm-poky-linux-gnueabi-strip
export RANLIB=arm-poky-linux-gnueabi-ranlib
export OBJCOPY=arm-poky-linux-gnueabi-objcopy
export OBJDUMP=arm-poky-linux-gnueabi-objdump
export AR=arm-poky-linux-gnueabi-ar
export NM=arm-poky-linux-gnueabi-nm
export M4=m4
export TARGET_PREFIX=arm-poky-linux-gnueabi-
export CONFIGURE_FLAGS=--target=arm-poky-linux-gnueabi --host=arm-poky-linux-gnueabi --build=i686-linux --with-libtool-sysroot=${SDKTARGETSYSROOT}
export CFLAGS= -O2 -pipe -g -feliminate-unused-debug-types
export CXXFLAGS= -O2 -pipe -g -feliminate-unused-debug-types
export LDFLAGS=-Wl,-O1 -Wl,--hash-style=gnu -Wl,--as-needed
export CPPFLAGS=
export KCFLAGS=--sysroot=${SDKTARGETSYSROOT}
export OECORE_DISTRO_VERSION=1.8
export OECORE_SDK_VERSION=1.8
export ARCH=arm
export CROSS_COMPILE=arm-poky-linux-gnueabi-
#### end of POKY exports

ifeq ($(DEBUG),1)
CFLAGS 		= -g -fPIC
else
CFLAGS 		= -O3 -fPIC
endif

LIBS		= -lm -lasound -ljpeg -lpthread -lv4l2

export SHELL CC AR LD SYS_INCLDES BUILD AS
export LIBNAME LIBVER TARGET_SYSTEM
export KB_FLAGS

#---------------------------------------------------------------------
# Rules
#---------------------------------------------------------------------
all: 	${LIBNAME}
	@cd tests && make -w
	@cd utils && make -w
 
doc: docs
docs:
	doxygen

${LIBNAME}: ${OBJS}
	@echo "Targetting Khepera Library for ${TARGET_SYSTEM}"
	@echo "Building ${LIBNAME}.so.${LIBVER}"
ifeq (${DEBUG},1)
	@echo "DEBUG MODE"
else
	@echo "RELEASE MODE"
endif
	@mkdir -p ${BUILD}/lib
	@rm -f ${BUILD}/lib/${LIBNAME}*
	@mkdir -p ${BUILD}/include/khepera
	$(CC) -o ${BUILD}/lib/${LIBNAME}.so.${LIBVER} -Wl,-soname,${LIBNAME}.so -shared ${OBJS} $(LIBS)
	@echo "Building ${LIBNAME}.a"
	@$(AR) r ${BUILD}/lib/${LIBNAME}.a ${OBJS}
	@echo "Adding includes"
	@cp *.h ${BUILD}/include/khepera
	@echo "Creating symlink ${LIBNAME}.so"
	@rm -f ${BUILD}/lib/${LIBNAME}.so
	@cd ${BUILD}/lib &&  cp -s ${LIBNAME}.so.${LIBVER} ${LIBNAME}.so


clean: 
	@echo "Cleaning"
	@rm -f *.o *~ .depend core*
	@cd tests && make -w clean
	@cd utils && make -w clean

depend:	
	@echo "Building dependencies"
	@rm -f .depend
	@touch .depend
	@makedepend ${SYS_INCLUDES} -Y -f .depend ${SRCS}
	@cd tests && make -w depend
	@cd utils && make -w depend

%.o: %.c %.h
	@echo "Building $@"
	@$(CC) -c $(CFLAGS) $(KB_FLAGS) $< -o $@

.PHONY: all clean depend docs

ifeq (.depend,$(wildcard .depend))
include .depend 
endif

