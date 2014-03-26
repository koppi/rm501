PKG := SDL2_ttf SDL2_net

PKG_CFLAGS  := $(shell pkg-config --cflags $(PKG))
PKG_LDFLAGS := $(shell pkg-config --libs   $(PKG))

SDL_CFLAGS  := $(shell sdl2-config --cflags)
SDL_LDFLAGS := $(shell sdl2-config --libs)

CFLAGS  := -g -Wall $(SDL_CFLAGS) $(PKG_CFLAGS)
LDFLAGS := -Wl,--as-needed
LDLIBS  := $(SDL_LDFLAGS) $(PKG_LDFLAGS) -lcurses -lGL -lGLU -lm -lserialport

all: rm501

%.o: %.c
	$(CC) -o $@ -c $(CFLAGS) $+

rm501: rm501.o
	$(CC) -o $@ $+ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f *.o
	rm -f rm501
