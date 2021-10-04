INCDIR=-Iplatform -Iboards/icicle-kit-es/platform_config -Iboards/icicle-kit-es -Wall

CC=gcc
CFLAGS=$(INCDIR)

LIBDIR=.

LIBS=

_OBJ = main.o platform/drivers/mss/mss_can/mss_can.o
OBJ = $(_OBJ)

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

can-external-loopback: $(OBJ)
	$(CC) -o $@ $(OBJ) $(CFLAGS) $(LIBS)

.phony: clean

clean:
	rm -rf *.o *~ core  platform/drivers/mss_can/*~ platform/drivers/mss_can.o can-external-loopback
