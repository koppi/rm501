
![rm501.png](doc/rm501.png)


## Features

 * OpenGL gui,
 * forward and inverse kinematics (double precision math),
   * handles input from keyboard and SpaceNavigator(TM),
   * inter-process communication via UNIX domain socket:
     connect with:
```
$ socat READLINE,history=$HOME/.rm501_history TCP:127.0.0.1:8888,crlf
```
     or
```
$ nc localhost 8888 # if the above command does not work for you.
```

## Compile and run

Required libraries: OpenGL, SDL2, SDL2_ttf (-- not SDL_ttf2!)

```
$ sudo apt-get -y install mesa-common-dev libsdl2-dev libsdl2-net-dev
```

Download, build and install SDL2_ttf (it's not in the repos, yet):

```
$ wget http://www.libsdl.org/projects/SDL_ttf/release/SDL2_ttf-2.0.12.tar.gz
$ tar xvfz SDL2_ttf-2.0.12.tar.gz; cd SDL2_ttf-2.0.12
```

Build and install SDL2_ttf as a Debian package:

```
$ sudo apt-get -y install devscripts dh-autoreconf
$ debuild
$ sudo dpkg -i ../libsdl2-ttf*deb
```

Or build and install without building a Debian package:

```
$ ./autogen.sh; configure --prefix=/usr; make; sudo make install 
```

## Authors

https://github.com/koppi
