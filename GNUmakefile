BIN := rm501

# install prefix either /usr or /usr/local on most unix systems
PREFIX ?= /usr

ifeq ($(OS),Windows_NT)
  OPENGL_LIBS:=-lopengl32 -lglu32
else
  OPENGL_LIBS:=-lGL -lGLU
endif

# comment out to disable SDL GUI
SDL_CFLAGS := -DHAVE_SDL $(shell sdl2-config --cflags) $(shell pkg-config --cflags SDL2_ttf SDL2_image)
SDL_LDLIBS := $(shell sdl2-config --libs) $(shell pkg-config --libs SDL2_ttf SDL2_image) $(OPENGL_LIBS) -lpng

# comment out to disable Curses GUI
#CURSES_CLFAGS := -DHAVE_NCURSES
#CURSES_LDLIBS := -lncursesw

# comment out to disable LinuxCNC / Machinekit HAL component functionality
#HAL_CFLAGS := -DHAVE_HAL -I/usr/include/linuxcnc -DRTAPI -DTHREAD_FLAVOR_ID=RTAPI_POSIX_ID
#HAL_LDLIBS := -llinuxcnchal -lprotobuf

# comment out to disable ZeroMQ functionality
#ZMQ_CFLAGS := -DHAVE_ZMQ
#ZMQ_LDLIBS := -l:libzmq.so.5

# comment out to disable Eclipse MQTT functionality
#MQTT_CFLAGS := -DHAVE_MQTT
#MQTT_LDLIBS := -lpaho-mqtt3c
#MQTT_OBJS   := mqtt_handler.o

# comment out to disable trajectory calculation functionality
#TRAJGEN_CFLAGS := -DHAVE_TRAJGEN
#TRAJGEN_LDLIBS :=
#TRAJGEN_OBJS   := trajgen.o

CFLAGS  += $(SDL_CFLAGS) $(CURSES_CLFAGS) $(HAL_CFLAGS) $(ZMQ_CFLAGS) $(MQTT_CFLAGS) $(TRAJGEN_CFLAGS) -O2 -g -Wall -std=gnu11
CFLAGS  += -Wno-unused-variable
LDLIBS  += $(SDL_LDLIBS) $(CURSES_LDLIBS) $(HAL_LDLIBS) $(ZMQ_LDLIBS) $(MQTT_LDLIBS) $(TRAJGEN_LDLIBS) -lm
LDFLAGS += -Wl,--as-needed

OBJS     = $(BIN).o $(MQTT_OBJS) $(TRAJGEN_OBJS)

all: $(BIN)

DEPS := $(OBJS:.o=.d)
-include $(DEPS)

%.o: %.c
	$(CC) $(CFLAGS) -MD -c -o $@ $<

$(BIN): $(OBJS)
	$(CC) -o $@ $+ $(LDFLAGS) $(LDLIBS)

$(BIN).1: $(BIN)
	help2man -N ./$+ > $@

install: $(BIN)
	strip $(BIN)
	mkdir -p $(INSTALL_PREFIX)$(PREFIX)/bin
	install -m 0755 $(BIN)        $(INSTALL_PREFIX)$(PREFIX)/bin
	mkdir -p $(INSTALL_PREFIX)$(PREFIX)/share/icons/hicolor/512x512/apps
	install -m 0664 $(BIN).png     $(INSTALL_PREFIX)$(PREFIX)/share/icons/hicolor/512x512/apps
	mkdir -p $(INSTALL_PREFIX)$(PREFIX)/share/applications
	install -m 0664 $(BIN).desktop $(INSTALL_PREFIX)$(PREFIX)/share/applications

uninstall:
	rm $(INSTALL_PREFIX)$(PREFIX)/bin/$(BIN)
	rm $(INSTALL_PREFIX)$(PREFIX)/share/icons/hicolor/512x512/apps/$(BIN).png
	rm $(INSTALL_PREFIX)$(PREFIX)/share/applications/$(BIN).desktop

clean:
	rm -f *~ $(OBJS) $(DEPS) $(BIN)

pull:
	git pull origin master --rebase

push:
	git push origin HEAD:master

