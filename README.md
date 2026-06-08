Welcome to the Klipper project!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

The Klipper firmware controls 3d-Printers. It combines the power of a
general purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use the Klipper software.

Start by [installing Klipper software](https://www.klipper3d.org/Installation.html).

Klipper software is Free Software. See the [license](COPYING) or read
the [documentation](https://www.klipper3d.org/Overview.html). We
depend on the generous support from our
[sponsors](https://www.klipper3d.org/Sponsors.html).

This fork of Klipper is minimally modified to support 6 axes. The position is stored as `[x, y, z, e, a, b, c]` in order to support [3:] parsing. 

Testing for now is done on a Cartesian Ender 3, currently still with 3 axes, though that is hopefully going to change soon.

For testing, I mapped the rotation axes onto the raspberry pi GPIO. It is probably easier than making multiple kinematics files for each combanation. 