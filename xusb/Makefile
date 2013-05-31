CC = gcc -pipe
VERSION = 0.1
CFLAGS = -O2 -fno-exceptions -DXUSB_VERSION=\"$(VERSION)\" \
	-W -Wall -Wformat
LDFLAGS = -lusb-legacy
INCLUDE = /opt/local/include/libusb-legacy
DIST_DEST = xusbprog-$(VERSION)
LDUSB = /opt/local/lib/libusb-legacy
all: xusbprog

# NOTE: Also add sources to the "dist:" target!
xusbprog: xusb.o
	#$(CC) -L$(LDUSB) $(LDFLAGS) -I$(INCLUDE) xusb.o -o xusbprog

clean:
	-rm -f *.o

distclean: clean
	-rm -f xusbprog

dist:
	mkdir -p "$(DIST_DEST)"
	cp Makefile "$(DIST_DEST)"
	cp xusbdev.cc xusbdev.h "$(DIST_DEST)"
	cp xusbprog.cc "$(DIST_DEST)"
	tar -c "$(DIST_DEST)" | gzip -9 > "xusbprog-$(VERSION).tar.gz"
	rm -r "$(DIST_DEST)"

.c.o:
	$(CC) -c $(CFLAGS) -I$(INCLUDE) $<

xusb.o: xusb.c xusb.h
