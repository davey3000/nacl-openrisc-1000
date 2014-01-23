# NaCl/PNaCl OpenRISC 1000 Emulator

*Version:* 0.0.1

An experimental port of the JavaScript OpenRISC 1000 emulator to C++ using
Chrome's NaCl/PNaCl libraries, allowing it to be run in the Chrome browser or
Chrome OS.

The JavaScript version is available [here](https://github.com/s-macke/jor1k/).

## Limitations

Compared to the JavaScript emulator, this emulator currently has a few prominent
limitations:

* The Ethernet device is not emulated.
* The Touchscreen device is not emulated.
* There is a bug that causes image corruption when decoding JPEGs.

In terms of differences in emulated hardware, this emulator has 128MB of RAM by
default, a 200MB hard disk image and a 120 x 40 terminal window (1680x1050 or
higher screen resolution recommended).

## Building

### Requirements

* Pepper SDK (tested with Canary)
* BZip2 library (in the NaClPorts repository)
* Python (tested with 2.7.3, but should work with 2+)

### Build process

A suitable version of the Pepper SDK tools must be installed. Consult the
[Pepper SDK download](https://developers.google.com/native-client/sdk/download) page for details.

The environment variables `NACL_ARCH` and `NACL_SDK_ROOT` should be set
appropriately (the former indicating the target architecture, e.g. `pnacl`;
the latter indicating the path to the Pepper SDK).

The BZip2 library must be present in the Pepper SDK area. For information on
how to build and install this library, please consult the [NaClPorts
repository](http://code.google.com/p/naclports).

To make the development build and place the result under `./dist/`:

```
make
```

To remove all build files:

```
make clean
```

## License

The software is distributed under the terms of the GNU General Public License (version 3). See LICENSE.md for details of this license.

## Credits

* David Hart -- C++ NaCl/PNaCl porting and additions
* Sebastian Macke [simulationcorner.net](http://simulationcorner.net) -- [JavaScript OpenRISC 1000 emulator](https://github.com/s-macke/jor1k/) and the JavaScript emulated terminal used by this project
