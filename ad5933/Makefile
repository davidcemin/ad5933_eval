
VERSION 	:= 0.1
TARGET		:= ad5933_eval
DIST_DEST	:= ad5933-$(VERSION)

########################################################################
#Include dirs
XUSB_DIR			:= ../xusb
LIBUSB_INCLUDE_DIR	:= /opt/local/include/libusb-legacy
LIBUSB_LIB_DIR		:= /opt/local/lib/libusb-legacy

########################################################################
#flags
CFLAGS   := -O2 -W -Wall -Wmissing-declarations -Wmissing-prototypes -fno-exceptions -DXUSB_VERSION=\"$(VERSION)\" -Wformat
CINCLUDE := -I. -I$(XUSB_DIR) -I$(LIBUSB_INCLUDE_DIR)
CLIBDIR  :=
CLIBS    := -L$(LIBUSB_LIB_DIR) -lusb-legacy
DEPFLAGS := -MM

#CFLAGS   := -O2 -W -Wall -Werror -Wmissing-declarations -Wmissing-prototypes -fno-exceptions -DXUSB_VERSION=\"$(VERSION)\" -Wformat
########################################################################
#includes
INCLUDE    = ${CINCLUDE}
FLAGS      = ${CFLAGS}
LIBDIR     = ${CLIBDIR}
LIBS       = ${CLIBS}
LIBINCLUDE = $(LIBDIR)

########################################################################
#Setzen alle zusammen..
CC 		= gcc -pipe
LD       = ld
AR       = ar
ARFLAGS  = rvu
CMPFLAGS = ${FLAGS} ${INCLUDE}
LDFLAGS  = ${LIBS}

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

$(TARGET): $(OBJECTS)
	$(QUIET)echo Compiling executable file: $@
	$(QUIET)${CC} $(LDFLAGS) -o $@  $^ $(OBJS) $(LIBFILES)

#.c.o:
#	$(CC) -c $(CFLAGS) $(INCLUDE) $<

%.o: %.c
	$(QUIET)echo Creating object: $@
	$(QUIET)$(CC) $(CMPFLAGS) $(LIBS) -c $^

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
