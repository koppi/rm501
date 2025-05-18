CC=$(CROSS)gcc
LD=$(CROSS)ld
AR=$(CROSS)ar
PKG_CONFIG=$(CROSS)pkg-config
SDL_CONFIG=$(CROSS)sdl2-config

BIN := rm501

# install prefix either /usr or /usr/local on most unix systems
PREFIX ?= /usr

ifeq ($(OS),Windows_NT)
  OPENGL_LIBS:=-lopengl32 -lglu32
else
  OPENGL_LIBS:=-lGL -lGLU
endif

REQPKG=sdl2
REQPKG:=$(shell $(PKG_CONFIG) --exists $(REQPKG) && echo '$(REQPKG)')
ifneq ($(REQPKG),)
  SDL_CFLAGS := -DHAVE_SDL $(shell $(SDL_CONFIG) --cflags)
  SDL_LDLIBS := $(shell $(SDL_CONFIG) --libs) $(OPENGL_LIBS)
endif

REQPKG=SDL2_ttf
REQPKG:=$(shell $(PKG_CONFIG) --exists $(REQPKG) && echo '$(REQPKG)')
ifneq ($(REQPKG),)
  SDL_CFLAGS += $(shell $(PKG_CONFIG) --cflags $(REQPKG))
  SDL_LDLIBS += $(shell $(PKG_CONFIG) --libs $(REQPKG))
endif

REQPKG=SDL2_image
REQPKG:=$(shell $(PKG_CONFIG) --exists $(REQPKG) && echo '$(REQPKG)')
ifneq ($(REQPKG),)
  SDL_CFLAGS += $(shell $(PKG_CONFIG) --cflags $(REQPKG))
  SDL_LDLIBS += $(shell $(PKG_CONFIG) --libs $(REQPKG)) # -lpng
endif

REQPKG=libpng
REQPKG:=$(shell $(PKG_CONFIG) --exists $(REQPKG) && echo '$(REQPKG)')
ifneq ($(REQPKG),)
  SDL_CFLAGS += -DHAVE_PNG $(shell $(PKG_CONFIG) --cflags $(REQPKG))
  SDL_LDLIBS += $(shell $(PKG_CONFIG) --libs $(REQPKG))
endif

REQPKG=ncursesw
REQPKG:=$(shell $(PKG_CONFIG) --exists $(REQPKG) && echo '$(REQPKG)')
ifneq ($(REQPKG),)
  CURSES_CFLAGS := -DHAVE_NCURSES $(shell $(PKG_CONFIG) --cflags $(REQPKG))
  CURSES_LDLIBS := $(shell $(PKG_CONFIG) --libs $(REQPKG))
endif

# comment out to disable LinuxCNC / Machinekit HAL component functionality
#HAL_CFLAGS := -DHAVE_HAL -I/usr/include/linuxcnc -DRTAPI -DTHREAD_FLAVOR_ID=RTAPI_POSIX_ID
#HAL_LDLIBS := -llinuxcnchal -lprotobuf

# comment out to disable ZeroMQ functionality
#ZMQ_CFLAGS := -DHAVE_ZMQ
#ZMQ_LDLIBS := -l:libzmq.so.5

# comment out to disable Eclipse MQTT functionality
REQPKG=libmosquitto
REQPKG:=$(shell $(PKG_CONFIG) --exists $(REQPKG) && echo '$(REQPKG)')
ifneq ($(REQPKG),)
  MQTT_CFLAGS := -DHAVE_MQTT
  MQTT_LDLIBS := -lpaho-mqtt3c
  MQTT_OBJS   := mqtt_handler.o
endif

# comment out to disable trajectory calculation functionality
TRAJGEN_CFLAGS := -DHAVE_TRAJGEN
TRAJGEN_LDLIBS :=
TRAJGEN_OBJS   := trajgen.o

CFLAGS  += $(SDL_CFLAGS) $(CURSES_CFLAGS) $(HAL_CFLAGS) $(ZMQ_CFLAGS) $(MQTT_CFLAGS) $(TRAJGEN_CFLAGS) -O2 -g -Wall -std=gnu11
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
	git pull origin main --rebase

push:
	git push origin HEAD:main

