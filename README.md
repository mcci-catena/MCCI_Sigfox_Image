# MCCI Sigfox/STM32 Image library for Arduino

This repository holds the Sigfox and ST image libraries needed for supporting Sigfox under the Arduino environment.

At least with build system 1.8.12, we found that you could put `.h` files and `.a` files into precompiled libraries, but not `.cpp` files. Hence the existence of this library separate from the `MCCI_Arduino_Sigfox` library, which contains the C++ wrappers and C code for the IDE.

Please refer to `MCCI_Arduino_Sigfox` for the top level.

## Meta

The precompiled contents of this library are distributed under the licenses granted by Sigfox ([here](./License Sigfox library to ST Customers _ legal.txt)) and ST ([here](./st_readme.txt)). MCCI contents are distributed under the MIT license.