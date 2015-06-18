
![rm501.png](doc/rm501.png)


## Features

 * OpenGL gui,
 * ncurses gui (WIP),
 * controll from keyboard, JoyPad or SpaceNavigator,
 * forward and inverse kinematics (double precision math),
 * inter-process communication via UNIX domain socket (WIP).
```
$ socat READLINE,history=$HOME/.rm501_history TCP:127.0.0.1:8888,crlf
```
     or
```
$ nc localhost 8888 # if the above command does not work for you.
```

## Compile and run

Required libraries: OpenGL, SDL2, SDL2_ttf (-- not SDL_ttf2!)

```bash
$ sudo apt-get -y install mesa-common-dev libsdl2-dev libsdl2-ttf-dev libsdl2-net-dev
```

```bash
$ make
$ ./rm501
```

## Authors

https://github.com/koppi
