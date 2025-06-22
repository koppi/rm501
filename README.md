# Mitsubishi RM-501 Movemaster II Robot Simulator

The RM501 MoverMaster is a robot arm model from Mitsubishi Electric that was popular in the 1980s. It was developed for various industrial applications such as assembly, handling, and inspection.

Some of the key features of the RM501 MoverMaster are:

- **Five axes**: The robot arm has five axes that allow flexible movement and positioning.
- **Industrial applications**: The RM501 was developed for various industrial tasks such as assembly, handling, and inspection.
- **Programmability**: The robot can be programmed to perform complex tasks.

The RM501 MoverMaster was an important step in the development of industrial robots and is still used in some applications today.

[![Mitsubishi RM-501 Movemaster II Robot Simulator](https://markdown-videos-api.jorgenkh.no/url?url=https%3A%2F%2Fwww.youtube.com%2Fwatch%3Fv%3DdLeDPIRKhOw)](https://www.youtube.com/watch?v=dLeDPIRKhOw)

## Features

Working
 * [OpenGL](https://www.opengl.org) GUI,
 * control from [computer keyboard](https://en.wikipedia.org/wiki/Computer_keyboard), [Gamepad](https://en.wikipedia.org/wiki/Gamepad) or [3D mouse](https://en.wikipedia.org/wiki/3Dconnexion),
 * [forward](https://en.wikipedia.org/wiki/Forward_kinematics) and [inverse](https://en.wikipedia.org/wiki/Inverse_kinematics) kinematics ([double precision math](https://en.wikipedia.org/wiki/Double-precision_floating-point_format)),
 * implemented in four [C](https://en.wikipedia.org/wiki/C_(programming_language)) files for simplicity and portability,

Unfinished
 * Stefan Wilhelm's [trajectory planner](https://atwillys.de/content/cc/trajectory-generator-in-c),
 * Command-line [ncurses](https://www.gnu.org/software/ncurses) TUI,
 * Control from [LinuxCNC](https://www.linuxcnc.org) via [HAL](https://linuxcnc.org/docs/html/hal/intro.html).

## Clone, compile, install and run

```bash
sudo apt -y install git 
git clone https://github.com/koppi/rm501.git
```

* Required: [OpenGL](https://www.opengl.org), [SDL2](https://www.libsdl.org), [SDL2_ttf](https://www.libsdl.org/projects/SDL_ttf)
* Optional: [ncurses](https://www.gnu.org/software/ncurses), [libpng](https://www.libpng.org), [ZeroMQ](https://zeromq.org), [Eclipse Mosquitto™](https://mosquitto.org), [LinuxCNC HAL](https://linuxcnc.org/docs/html/hal/tutorial.html)

```bash
# required packages
sudo apt -y install mesa-common-dev libsdl2-dev libsdl2-ttf-dev libsdl2-image-dev freeglut3-dev
sudo usermod -aG input $USER            # for joystick access
sudo apt -y install fonts-dejavu-core   # for DejaVuSansMono.ttf
# optional packages
sudo apt -y install libncursesw5-dev    # for console
sudo apt -y install libpng-dev          # for screenshots
sudo apt -y install libzmq5 libzmq3-dev # for ZeroMQ
sudo apt -y install libpaho-mqtt-dev    # for Eclipse MQTT
sudo apt -y install mosquitto           # for Mosquitto MQTT server
sudo apt -y install linuxcnc-uspace-dev # for LinuxCNC
```

Compile and run:

```bash
cd rm501/
make
./rm501 -s # start program with SDL GUI
```

[![windows](../../actions/workflows/windows.yml/badge.svg)](../../actions/workflows/windows.yml) [![.github/workflows/ubuntu.yml](../../actions/workflows/ubuntu.yml/badge.svg)](../../actions/workflows/ubuntu.yml).

## Control

* Via keyboard

| Key on keyboard | Emulator key       |
| --------------- | ------------------ |
| <kbd>⎋ Escape</kbd> / <kbd>ctrl-c</kbd> | quit the simulator |
| <kbd>⇥ Tab</kbd> | toggle 3D view |
| <kbd>Q</kbd>, <kbd>W</kbd>, <kbd>E</kbd>, <kbd>R</kbd>, <kbd>T</kbd>   | increase joint value for waist, shoulder, elbow, pitch and roll |
| <kbd>A</kbd>, <kbd>S</kbd>, <kbd>D</kbd>, <kbd>F</kbd>, <kbd>G</kbd>   | decrease joint value for waist, shoulder, elbow, pitch and roll |
| <kbd>I</kbd> <kbd>K</kbd>, and direction keys (on your right-hand side) | control the end-effector position |
| <kbd>C</kbd>, <kbd>V</kbd>            | move end effector in a circle in the x-y plane, reset circle position |
| <kbd>N</kbd>, <kbd>H</kbd>            | move to 'nest' position, move to the 'all link horizontal' home position |
| <kbd>O</kbd>, <kbd>L</kbd>            | open, close tool |
  
  Hold the shift key to move the joints in parallel, not linearly.
  
* Via SpaceNavigator, see ```HAVE_SPACENAV```.
  
* Via Gamepad, see ```HAVE_JOYSTICK```.
  
* Via ØMQ, unfinished (see ```HAVE_ZMQ```).

* Via Eclipse MQTT, partially working (see ```HAVE_MQTT```).

* Via LinuxCNC, unfinished (see ```HAVE_HAL```).

## Authors

* **Jakob Flierl** - [koppi](https://github.com/koppi)
* **Stefan Wilhelm** - [Willy](https://atwillys.de/content/cc/trajectory-generator-in-c)
