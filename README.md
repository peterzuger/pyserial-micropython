# pyserial-micropython

## Table of Contents
+ [About](#about)
+ [Getting Started](#getting_started)
+ [Usage](#usage)
+ [module documentation](https://pyserial.readthedocs.io/en/latest/)

## About <a name = "about"></a>
This is a port of [pySerial][2] for the unix port of [micropython][1].

For the documentation of the `pySerial` module see here:
[readthedocs](https://pyserial.readthedocs.io/en/latest/).

## Getting Started <a name = "getting_started"></a>

### Prerequisites
This port of [pySerial][2] is designed for [micropython][1].
```
git clone --recurse-submodules https://github.com/micropython/micropython.git
```

### Installing
[pyserial-micropython](https://github.com/peterzuger/pyserial-micropython) will
very likely only work on the unix port of [micropython][1].

First create a modules folder next to your copy of [micropython][1].
```
project/
├── manifest.py
├── modules/
│   └──pyserial-micropython/
│       ├──...
│       └──manifest.py
└── micropython/
    ├──ports/
   ... ├──stm32/
      ...
```

And now put this project in the modules folder.
```
cd modules
git clone https://gitlab.com/peterzuger/pyserial-micropython.git
```

This module is implemented in pure [micropython][1], to include this in your final
unix executable it has to be included via a custom `manifest.py` wich could
reside in the top level directory above the `modules` and the `micropython`
folder like shown in the example tree above.

This custom `manifest.py` could look like this if only this custom module will
be included in the final binary:
```
include("modules/pyserial-micropython/manifest.py")
```

Now that you have everything that is needed, it is time to build
[micropython][1]. First the mpy-cross compiler has to be built:
```
make -C micropython/mpy-cross
```

Now compile the unix port and configure your custom `FROZEN_MANIFEST` like this:
```
make -C micropython/ports/unix FROZEN_MANIFEST=$PWD/manifest.py
```

and you are ready to use serial for [micropython][1].

## Usage <a name = "usage"></a>
The module is available by just importing serial:
```
import serial

port = serial.Serial("/dev/ttyUSB0", 115200)
```

[1]: https://github.com/micropython/micropython
[2]: https://github.com/pyserial/pyserial/
