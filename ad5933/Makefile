
VERSION 	:= 0.1
TARGET		:= ad5933_eval
DIST_DEST	:= ad5933-$(VERSION)
PLAT		:=

########################################################################
#Include dirs
XUSB_DIR			:= ../xusb
LIBUSB_INCLUDE_DIR	:= /opt/local/include/libusb-legacy
LIBUSB_LIB_DIR		:= /opt/local/lib/libusb-legacy

########################################################################
#flags
CFLAGS   	:= -O2 -W -Wall -Wmissing-declarations -Wmissing-prototypes -fno-exceptions -DXUSB_VERSION=\"$(VERSION)\" -Wformat
CINCLUDE 	:= -I. -I$(XUSB_DIR) 
CLIBDIR  	:=
CLIBS_MAC	:= -L$(LIBUSB_LIB_DIR) -lusb-legacy
CLIBS_LINUX	:= -lusb
DEPFLAGS	:= -MM

#CFLAGS   := -O2 -W -Wall -Werror -Wmissing-declarations -Wmissing-prototypes -fno-exceptions -DXUSB_VERSION=\"$(VERSION)\" -Wformat
########################################################################
#includes
INCLUDE_LINUX	:= ${CINCLUDE} -I/usr/include
INCLUDE_MAC		:= ${CINCLUDE} -I$(LIBUSB_INCLUDE_DIR)
FLAGS			:= ${CFLAGS}
LIBDIR			:= ${CLIBDIR}
LIBS_MAC		:= ${CLIBS_MAC}
LIBS_LINUX		:= $(CLIBS_LINUX)
LIBINCLUDE		:= $(LIBDIR)

########################################################################
#Setzen alle zusammen..
CC 		= gcc -pipe
LD       = ld
AR       = ar
ARFLAGS  = rvu
ifeq ($(PLAT),linux)
	CMPFLAGS = ${FLAGS} ${INCLUDE_LINUX}
	LDFLAGS  = ${LIBS_LINUX}
else
	CMPFLAGS = ${FLAGS} ${INCLUDE_MAC}
	LDFLAGS  = ${LIBS_MAC}
endif

########################################################################
CSOURCES = $(wildcard *.c)
OBJECTS  = $(CSOURCES:.c=.o)
DEPS     = $(CSOURCES:.c=.d)
OBJS	 = $(XUSB_DIR)/*.o
LIBFILES =

#General
QUIET = @

.PHONY: clean clobber dist

all: $(TARGET)

$(TARGET): stats.o ad5933_utils.o ad5933.o
	$(QUIET)echo Compiling executable file: $@
	$(QUIET)${CC} $(LDFLAGS) -o $@  $^ $(OBJS) $(LIBFILES)

#.c.o:
#	$(CC) -c $(CFLAGS) $(INCLUDE) $<

%.o: %.c
	$(QUIET)echo Creating object: $@
ifeq ($(PLAT),linux)
	$(QUIET)$(CC) $(CMPFLAGS) $(LIBS_LINUX) -c $^
else
	$(QUIET)$(CC) $(CMPFLAGS) $(LIBS_MAC) -c $^
endif

clean:
	$(QUIET)echo Removing temp and object files
	$(QUIET)rm -f *~ *.bak *.o $(DEPS) 

clobber: clean
	$(QUIET)echo Removing executable and some unnecessary files
	$(QUIET)rm -f $(TARGET)


#all: ad5933
#
## NOTE: Also add sources to the "dist:" target!
#ad5933: ad5933.o
#	$(CC) -L$(LDUSB) $(LDFLAGS) $(INCLUDE) $(OBJINC) -o ad5933
#
#clean:
#	-rm -f *.o
#
#distclean: clean
#	-rm -f ad5933
#

#xusb.o: xusb.c xusb.h
