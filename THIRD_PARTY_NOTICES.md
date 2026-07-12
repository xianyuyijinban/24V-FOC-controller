# Third-Party Notices

The root MIT license applies to original code and documentation authored for
this project. Third-party components remain subject to their own licenses.
Copyright notices and license files supplied by those projects must be kept
intact when their source or binaries are redistributed.

## Firmware dependencies

### STM32H7 HAL driver

- Location: `Drivers/STM32H7xx_HAL_Driver/`
- Upstream: STMicroelectronics
- License: BSD-3-Clause when received outside its original software package
- License text: `Drivers/STM32H7xx_HAL_Driver/LICENSE.txt`

### CMSIS and STM32H7 device support

- Location: `Drivers/CMSIS/`
- Upstream: Arm and STMicroelectronics
- License: Apache-2.0, subject to the notices shipped with each component
- License texts:
  - `Drivers/CMSIS/LICENSE.txt`
  - `Drivers/CMSIS/Device/ST/STM32H7xx/LICENSE.txt`

STM32CubeMX-generated files under `Core/` retain their STMicroelectronics
copyright and package-license notices.

## Desktop application dependencies

The Python packages below are dependencies, not relicensed by this project.
Exact versions are selected by the Python environment unless a release build
pins them explicitly.

### PySide6 and Qt for Python

- Used by: `HostComputer/`, `FOC_Device_Bridge/`
- License offered by the installed package: LGPL-3.0-only OR GPL-2.0-only OR
  GPL-3.0-only; commercial terms may also be available from The Qt Company
- Project: <https://doc.qt.io/qtforpython-6/>

Binary distributors must include the applicable Qt for Python and Qt license
notices and comply with the terms for the particular modules they ship. The
source repository does not vendor PySide6 or Qt.

### pyqtgraph

- Used by: `HostComputer/`
- License: MIT
- Project: <https://www.pyqtgraph.org/>

### pySerial

- Used by: `HostComputer/`, `FOC_Device_Bridge/`
- License: BSD-3-Clause
- Project: <https://github.com/pyserial/pyserial>

### NumPy

- Used by: `HostComputer/`
- License: BSD-3-Clause, with additional notices for bundled third-party code
- Project: <https://numpy.org/>

### PyInstaller

- Used only to build Windows application packages
- License: GPL-2.0-or-later with the PyInstaller bootloader exception
- Project: <https://pyinstaller.org/>

## Referenced projects

`Ctrl-FOC-Lite` was consulted as a design reference. No source code from that
repository is included or redistributed as part of this project.
