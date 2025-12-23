A cross-platform simulator for the iconic Mitsubishi RM-501 Movemaster II robot arm, popular in the 1980s for industrial assembly, handling, and inspection tasks.

[![Build: Windows](../../actions/workflows/windows.yml/badge.svg)](../../actions/workflows/windows.yml)
[![Build: Ubuntu](../../actions/workflows/ubuntu.yml/badge.svg)](../../actions/workflows/ubuntu.yml)

## Overview

- **Five-axis robot arm simulation**
- Interactive OpenGL GUI
- Real-time keyboard, gamepad, and 3D mouse control
- Forward and inverse kinematics
- Simple and portable C codebase

[![YouTube Demo](https://markdown-videos-api.jorgenkh.no/url?url=https%3A%2F%2Fwww.youtube.com%2Fwatch%3Fv%3DdLeDPIRKhOw)](https://www.youtube.com/watch?v=dLeDPIRKhOw)

---

## Features

**Working:**
- OpenGL GUI
- Control via keyboard, gamepad, or 3D mouse
- Double-precision forward/inverse kinematics
- Minimal C source files for easy portability

**Experimental/Unfinished:**
- Trajectory planner (by Stefan Wilhelm)
- Command-line ncurses TUI
- LinuxCNC HAL integration
- Messaging: ØMQ, Eclipse MQTT, and others

---

## Getting Started

### Prerequisites

- **Required:** OpenGL, SDL2, SDL2_ttf
- **Optional:** ncurses, libpng, ZeroMQ, Eclipse Mosquitto, LinuxCNC HAL

### Installation (Debian/Ubuntu):

```bash
sudo apt -y install git mesa-common-dev libsdl2-dev libsdl2-ttf-dev libsdl2-image-dev freeglut3-dev
sudo apt -y install fonts-dejavu-core
sudo apt -y install libncursesw5-dev libpng-dev libzmq5 libzmq3-dev libpaho-mqtt-dev mosquitto linuxcnc-uspace-dev
sudo usermod -aG input $USER # joystick access
git clone https://github.com/koppi/rm501.git
```

### Compile & Run

```bash
cd rm501
make
./rm501
```

---

## Controls

| Key(s)                               | Action                                |
|-------------------------------------- |---------------------------------------|
| Esc / Ctrl-C                         | Quit                                  |
| Tab                                  | Toggle 3D view                        |
| Q/W/E/R/T, A/S/D/F/G                 | +/- joint values (waist, shoulder, etc.) |
| I/K, arrow keys                      | Move end-effector                     |
| C/V                                  | Circle motion / Reset circle          |
| N/H                                  | Move to nest/home positions           |
| O/L                                  | Open/close tool                       |
| Shift                                | Move all joints in parallel           |

- **SpaceNavigator:** Enable `HAVE_SPACENAV`
- **Gamepad:** Enable `HAVE_JOYSTICK`
- **Messaging (MQTT/ØMQ):** Enable `HAVE_MQTT` / `HAVE_ZMQ`
- **LinuxCNC:** Enable `HAVE_HAL`

---

## Contributors

- [Jakob Flierl (koppi)](https://github.com/koppi)
- [Stefan Wilhelm (trajectory planner)](https://atwillys.de/content/cc/trajectory-generator-in-c)

---

## License

LGPL-2.1-or-later. See [LICENSE](LICENSE).
