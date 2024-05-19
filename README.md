# mppt-esc

MPPT for a solar-powered model glider, acting on the throttle (ESC signal).



## Overview
Based on [Sparkfun Arduino Pro Micro 3.3V/8MHz](https://www.sparkfun.com/products/12587), this
throttle-only MPPT sits between the receiver and the ESC. It reads one (optionally two) signals
from the receiver, as well as the S/A array voltage and adjusts the throttle following a
user-defined algorithm.

The Arduino, receiver and servos may be supplied directly from the S/A, depending on HW configuration
with a jumper.

Features:
- Up to two input signals from the receiver with microsecond resolution
- Throttle signal output with microsecond resolution
- Input and output signals constrained within valid pusle length range
- S/A voltage measurement up to ca. 10V
- Exponential-moving average filter to S/A voltage to reduce noise
- Regular acquisition and filtering of S/A voltage in the background
- Regular execution of used-defined MPPT controller update function
- Adjustable start-up delay to allow ESC to initialize properly



## Warning!

This hardware and software are intended to go in between the receiver and the ESC. This means
if things go wrong, the propeller may spin up unexpectedly and/or cannot be stopped anymore.
THIS MAY BE DANGEROUS.

Therefore:
- use this only if you really know what you are doing!
- use this only if you know what to do in case things go wrong, to prevent harm to yourself,
and anybody round you!
- don't perform firmware updates while the ESC is connected and/or powered on!

For the formal legal disclaimer, see the [license file](LICENSE).



## Installation

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (currently this is version 2.x)
2. Install the Sparkfun AVR board drivers and packages in the the IDE following this
[Sparkfun tutorial](https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide#installing-windows).<br>
Note that the tutorial is still for the older v1.x IDE, the concept is however still the same.
3. Clone repository: `git clone https://github.com/falthaus/mppt-esc.git`.<br>
   If you don't have git installed, alternatively download ZIP file from Github.
4. Open repository files in the Arduino IDE
5. In the IDEs "Tools" menu, select "Sparkfun AVR Boards" - "Sparkfun Pro Micro" as the board.
6. Also in the "Tools" menu, select "ATmega32U4 (3.3V, 8 MHz)" as the processor.
7. Finally, also in the "Tools" menu, select the correct "COMx" port the board is connected to.
8. Connect hardware, compile and upload



## Firmware

### Files
Hardware-related and background functionality and user-defined MPPT logic are kept in separate files:
- `mppt-esc.ino`: Main "sketch" with background and hardware-related functionality.<br>
   Generally this doesn't need to be modified by the user
- `mppt.cpp`: User-defined MPPT initialization `mppt_init()` and update `mppt_update()` functions
- `mppt.h`: User-defined MPPT configuration parameters

### Configuration
Configuration parameters that may be modified by the user are in `mppt.h`:

|Name|Value|Unit|Description|
|:---|:---:|:---:|:---|
|`RCMIN_US`|`1000`|[us] |minimum valid input pulse length|
|`RCMAX_US`|`2000`| [us] |maximum valid input pulse length|
|`THRMIN_US`|`1000`| [us] |minimum valid throttle pulse length|
|`THRMAX_US`|`2000`| [us] |maximum valid throttle pulse length|
|`INIT_WAIT_MS`|`5000`| [ms] |time to wait for ESC initialization|
|`V_MPPT_MV`|`7000`| [mV] |target MPPT S/A voltage|
|`THR_INC_US`|`16`| [us] |incremental change to throttle signal|
|`F_CTRL_HZ`|`10`| [Hz] |MPPT control frequency|

Some hardware-specific parameters and constants are in `mppt.cpp`. Normally these do not need to be
modified by the user.



## Hardware
See the [HW documentation](hw/).
