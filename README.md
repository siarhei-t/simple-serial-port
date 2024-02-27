# simple-serial-port
A small library written in C++ for working with a serial port with limited configuration options, designed to work with chips such as ft232, ch340, etc. Can be built for Windows and Linux.

## Building
In the example, configuration is done using Cmake. Ninja is used as the default build tool. However, any other build tool can also be used.

**Configure for Windows:** 

```sh
cmake -DTARGET_WINDOWS=ON -Bbuild
```
**Configure for Linux:** 

```sh
cmake -DTARGET_LINUX=ON -Bbuild
```

**Building:**
```sh
cmake --build build
```