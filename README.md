
![rm501.png](doc/rm501.png)

## Features

Working features:

 * [OpenGL](https://www.opengl.org/) GUI,
 * control from keyboard, [JoyPad](https://www.google.com/?q=joypad) or [SpaceNavigator](https://www.google.com/?q=spacenavigator),
 * [forward](https://en.wikipedia.org/wiki/Forward_kinematics) and [inverse](https://en.wikipedia.org/wiki/Inverse_kinematics) kinematics ([double precision math](https://en.wikipedia.org/wiki/Double-precision_floating-point_format)),
 * implemented in one [C](https://en.wikipedia.org/wiki/C_(programming_language)) file for simplicity and portability.

Untested / unfinished features:
 
 * [ncurses](https://www.gnu.org/software/ncurses/) UI,
 * control from network ([inter-process communication](https://en.wikipedia.org/wiki/Inter-process_communication) via [UNIX socket](https://en.wikipedia.org/wiki/Unix_domain_socket)),
 * control from [LinuxCNC](http://www.linuxcnc.org/) or [Machinekit](http://www.machinekit.io/) via [HAL](http://linuxcnc.org/docs/html/hal/intro.html).

## Video

[![Mitsubishi RM-501 Movemaster II Robot Simulator](http://img.youtube.com/vi/ddvIzk9aeJo/0.jpg)](https://www.youtube.com/watch?v=ddvIzk9aeJo)

## Compile and run

* Required libraries: [OpenGL](https://www.opengl.org/), [SDL2](https://www.libsdl.org/), [SDL2_ttf](https://www.libsdl.org/projects/SDL_ttf/)
* Optional libraries: [ncurses](https://www.gnu.org/software/ncurses/), [libpng](http://www.libpng.org/), [LinuxCNC / Machinekit HAL](http://linuxcnc.org/docs/html/hal/tutorial.html)

```bash
$ sudo apt-get -y install git 
$ git clone https://github.com/koppi/rm501.git
```

```bash
$ sudo apt-get -y install mesa-common-dev libsdl2-dev libsdl2-ttf-dev libsdl2-net-dev
$ sudo apt-get -y install libncurses-dev  libpng-dev
$ sudo apt-get -y install ttf-dejavu-core # for DejaVuSansMono.ttf
```

```bash
$ cd rm501/
$ make
$ ./rm501
```

## Control

* Control via keyboard:
  * Q, W, E, R, T – increase the joint values for waist, shoulder, elbow, pitch and roll,
  * A, S, D, F, G – decrease the joint values for waist, shoulder, elbow, pitch and roll,
  * I, K, and direction keys (on your right-hand side): control the end-effector position.
  
* Control via SpaceNavigator
  * see ```HAVE_SPACENAV``` in source code.
  
* Control via JoyPad
  * see ```HAVE_JOYSTICK``` in source code.
  
* Control via network:

```bash
$ socat READLINE,history=$HOME/.rm501_history TCP:127.0.0.1:8888,crlf
```
or
```bash
$ nc localhost 8888 # if the above command does not work for you.
```

Unfinished (see ```HAVE_SOCKET``` in source code).

* Control via LinuxCNC / Machinekit:

Unfinished (see ```HAVE_HAL``` in source code).

## Authors

https://github.com/koppi
