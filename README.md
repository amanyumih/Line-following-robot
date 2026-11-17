# Line-following robot

Firmware for a line-following robot built on an STM32 "Blue Pill" board (STM32F103C8), written in C++
with the Arduino framework and PlatformIO. The robot was planned for a rescue-line course: follow a black
line, turn where green markers show the way, go around obstacles and climb a ramp.

This is an early version, from 2019. The pin map and the state machine are in place, but most of the
behaviours are still empty. The comments in the code are in Portuguese.

## How it works

The robot runs a finite-state machine, in `loop()` in `src/main.cpp`.

| State | What it does | In the code |
|---|---|---|
| `VERIFICA` | Reads the sensors and picks the next state | Done; the ramp and obstacle checks still always return 0 |
| `LINHA` | Follows the line and drives straight across intersections | Partly done |
| `ESQ90`, `DIR90` | Turns 90° to the left or to the right | Empty |
| `RET180` | Turns around | Empty |
| `CONTORNA` | Goes around an obstacle | Not in the state machine yet |
| `RAMPA` | Climbs the ramp; the rescue states come after it | Empty |

At an intersection, the two green sensors choose the way: green on the left turns left, green on the
right turns right, and green on both sides turns around. Right after an intersection the green sensors are
ignored, so the robot does not read the same marker twice.

## Wiring

| Part | Pins |
|---|---|
| Line sensors: far left, left, centre, right, far right | PB10, PA5, PA4, PB14, PB15 |
| Green-marker sensors: left, right | PA15, PB3 |
| Right wheel motor: clockwise, counter-clockwise | PB8, PB9 |
| Left wheel motor: clockwise, counter-clockwise | PA0, PA1 |
| Servos (pins not final) | PB0, PB1 |
| Wheel encoders (DT, CLK) | PA8, PA9 and PA6, PA7 |
| Ultrasonic sensor: trigger, echo (echo must be 5 V tolerant) | PB12, PB13 |
| Built-in LED | PC13 |

Planned but commented out: an I2C display (SDA on PB7, SCL on PB6) and an HC-05 Bluetooth module (TX on
PA2, RX on PA3). The top of `src/main.cpp` lists which Blue Pill pins are 5 V tolerant, PWM, serial and
ADC.

## Build and upload

1. Install [PlatformIO](https://platformio.org/) (the VS Code extension is the easiest way) and open this
   folder.
2. Build with `pio run`.
3. Upload over USB with `pio run -t upload`. It uses the DFU bootloader: keep the board in reset until the
   upload starts looking for the DFU device.
4. Open the serial monitor at 115200 baud. `platformio.ini` points to COM17; change it to your port.

The STM32 platform is pinned to version 4.5.x because, as a note in `platformio.ini` says, newer versions
may break the USB bootloader. To upload through a USB-serial adapter (an FTDI or an Arduino Nano), use the
commented `upload_protocol = serial` lines instead.

## Next steps

- Write the turn, turn-around, obstacle and ramp states.
- Implement `verifica_acc()` (ramp) and `verifica_obst()` (obstacle).
- Set the ultrasonic trigger pin (`TRIG`) as an output; it is configured as an input.
- In `LINHA`, drive the left motor too: the code writes to the right motor twice.
- Add the line-alignment correction marked in `LINHA`.
