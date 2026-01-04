FROM emscripten/emsdk
RUN apt update
RUN apt install -y ca-certificates gpg wget
RUN test -f /usr/share/doc/kitware-archive-keyring/copyright || wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
RUN apt update
RUN apt upgrade -y
RUN apt install -y ninja-build git cmake libsdl2-dev libsdl2-ttf-dev libsdl2-image-dev

WORKDIR /src

COPY . .

RUN cmake --version
RUN cmake --preset emscripten
RUN cmake --build --preset emscripten -j $(($(nproc) + 1))

FROM scratch
COPY --from=0 [ \
    "/src/build/emscripten/index.html", \
    "/src/build/emscripten/rm501.js", \
    "/src/build/emscripten/rm501.wasm", \
    "/src/build/emscripten/favicon.ico", \
    "." \
]
