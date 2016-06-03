PKG := SDL2_ttf

PKG_CFLAGS := $(shell pkg-config --cflags $(PKG))
PKG_LDLIBS := $(shell pkg-config --libs   $(PKG))

SDL_CFLAGS := $(shell sdl2-config --cflags)
SDL_LDLIBS := $(shell sdl2-config --libs) -lGL -lGLU -lpng

CURSES_CLFAGS := 
CURSES_LDLIBS := -lcurses

#HAL_CFLAGS := -I/usr/include/linuxcnc -DRTAPI -DTHREAD_FLAVOR_ID=RTAPI_POSIX_ID
#HAL_LDLIBS := -llinuxcnchal

#ZMQ_CFLAGS :=
#ZMQ_LDLIBS := -l:libzmq.so.3

CFLAGS  := $(SDL_CFLAGS) $(CURSES_CLFAGS) $(PKG_CFLAGS) $(HAL_CFLAGS) $(ZMQ_CFLAGS) -O2 -g -Wall
LDLIBS  := $(SDL_LDLIBS) $(CURSES_LDLIBS) $(PKG_LDLIBS) $(HAL_LDLIBS) $(ZMQ_LDLIBS) -lm
LDFLAGS := -Wl,--as-needed

all: rm501

%.o: %.c
	$(CC) -o $@ -c $(CFLAGS) $+

rm501: rm501.o
	$(CC) -o $@ $+ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f *.o rm501
