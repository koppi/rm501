
# install prefix either /usr or /usr/local on most unix systems
PREFIX ?= /usr

# comment out to disable SDL GUI
SDL_CFLAGS := -DHAVE_SDL $(shell sdl2-config --cflags) $(shell pkg-config --cflags SDL2_ttf)
SDL_LDLIBS := $(shell sdl2-config --libs)   $(shell pkg-config --libs   SDL2_ttf) -lGL -lGLU -lpng

# comment out to disable Curses GUI
CURSES_CLFAGS := -DHAVE_CURSES
CURSES_LDLIBS := -lcurses

# comment out to disable LinuxCNC / Machinekit HAL component functionality
#HAL_CFLAGS := -DHAVE_HAL -I/usr/include/linuxcnc -DRTAPI -DTHREAD_FLAVOR_ID=RTAPI_POSIX_ID
#HAL_LDLIBS := -llinuxcnchal -lprotobuf

# comment out to disable ZeroMQ functionality
#ZMQ_CFLAGS := -DHAVE_ZMQ
#ZMQ_LDLIBS := -l:libzmq.so.3

CFLAGS  += $(SDL_CFLAGS) $(CURSES_CLFAGS) $(HAL_CFLAGS) $(ZMQ_CFLAGS) -O2 -g -Wall
LDLIBS  += $(SDL_LDLIBS) $(CURSES_LDLIBS) $(HAL_LDLIBS) $(ZMQ_LDLIBS) -lm
LDFLAGS += -Wl,--as-needed

all: rm501

%.o: %.c
	$(CC) -o $@ -c $(CFLAGS) $+

rm501: rm501.o
	$(CC) -o $@ $+ $(LDFLAGS) $(LDLIBS)

install: rm501
	strip rm501
	install -m 0755 rm501         $(PREFIX)/bin
	install -m 0755 rm501.png     $(PREFIX)/share/icons/hicolor/512x512/apps
	install -m 0755 rm501.desktop $(PREFIX)/share/applications

uninstall:
	rm $(PREFIX)/bin/rm501
	rm $(PREFIX)/share/icons/hicolor/512x512/apps/rm501.png
	rm $(PREFIX)/share/applications/rm501.desktop

clean:
	rm -f *.o rm501

pull:
	git pull origin master --rebase

push:
	git push origin HEAD:master
