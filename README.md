# `landing`

This is a basic simulated guidance computer running on an
Arduino using FreeRTOS as a preemptive RTOS.

# Wiring

Green LED goes to pin 2 and red LED goes to pin 3. Set up
the resistors, ground and voltage pins as you desire.

# Building

This requires the Arduino IDE to be installed on your
computer in addition to the FreeRTOS Arduino library and
the EEPROMEx library.

I have written a custom `make` target that automatically
selects the correct serial port to deploy to the Arduino.
You should modify the `CMakeLists.txt` to the correct
serial port as required, but you can run `make` manually
and use the command-line arguments specified by
Arduino-CMake-Toolchain if you desire.

``` shell
git clone https://github.com/AgentTroll/landing.git
cd landing
mkdir build && cd build
cmake .. && make deploy

# If you don't want to bother messing with the 
# CMakeLists.txt, you can manually specify the serial port

# make upload SERIAL_PORT=<desired port>
```

# Implementation

Most numerical constants utilize a `#define` rather than a
`static const` so as to avoid allocating values in program
memory space. `#define`d macros are inserted into the code,
which saves SRAM for the tasks.

String literals are all allocated in flash memory with the
`PROGMEM` quantifier for the task names and the `F()`
macro for in-code `String` objects.

The rest of the state values are `static` global values and
the remaining SRAM is taken up by the stack of each task.
These were measured and tuned using 
`uxTaskGetStackHighWaterMark()`.

The program is divided into 3 tasks, grouped by priority.
The mission clock task has the highest priority, followed
by the flight termination and guidance task and then
finally the low priority tasks, which includes command
handling, telemetry and status LED handling.

Procedurally, the program scans for 3 commands: 
`TRAJECTORY`, `BEGIN` and `SENSOR`. These commands load a
set of state vectors consisting of 3D position and velocity
vectors for each second of flight time, begins the
autonomous countdown and finally simulate sensor data,
respectively.

The guidance algorithm is a cascading PID loop that
attempts to first minimize position error and passes the
velocity to the velocity loop, which minimizes the
desired velocity and the output velocity error. The output
velocity is then sent through the serial port as
`TELEMETRY`.

The mission automatically ends after T+10 seconds.

# Notes

  * I've never written a guidance computer before. I have
  no idea what I'm writing. Don't use this in production,
  obviously.

# Credits

Built with [CLion](https://www.jetbrains.com/clion/)

Utilizes:
  * [Arduino-CMake-Toolchain](https://github.com/a9183756-gh/Arduino-CMake-Toolchain)
  * [FreeRTOS](https://www.freertos.org/)
    * [Arduino Library](https://www.arduino.cc/reference/en/libraries/freertos/)
  * [EEPROMEx](https://playground.arduino.cc/Code/EEPROMex/)
