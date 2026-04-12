# Racing_Rig Firmware

This is the motor controller firmware for the 2DOF racing rig, running on an Arduino UNO R3. It controls two 24V 75rpm motors via BTS7960 43A H-bridge drivers using a PID control loop, taking position feedback from AS5600 magnetic encoders on each axis.

## How It Works

The Arduino runs a 4kHz control loop that continuously reads the position of both motors, compares them to the target positions being sent from the PC, and adjusts the motor power accordingly using PID math. The faster and more accurately it can do this, the smoother the motion feels.

## Serial Commands

Connect via Serial Monitor at **500000 baud**. All commands use the format `[xxx]`.

| Command | Description |
|---|---|
| `[h]` | Print full command list |
| `[d]` | Run diagnostics |
| `[Axxx]` | Set left motor target (0-1023) |
| `[Bxxx]` | Set right motor target (0-1023) |
| `[ena]` | Enable both motors |
| `[en1]` / `[en2]` | Enable left / right motor individually |
| `[sav]` | Save all settings to EEPROM |
| `[ver]` | Print firmware version |
| `[mo1]` / `[mo2]` / `[mo0]` | Start / stop live telemetry |

## Pin Assignments

| Pin | Function |
|---|---|
| 9 | Left motor PWM |
| 10 | Right motor PWM |
| 2 / 3 | Left H-bridge ENA / ENB |
| 4 / 5 | Right H-bridge ENA / ENB |
| A0 | Left encoder feedback |
| A1 | Right encoder feedback |

## Safety Features

If the Arduino stops receiving commands from the PC for more than 15 seconds, it automatically reduces motor power as a precaution. Motors also instantly cut if the encoder reads outside the safe range, and on every power cycle both axes slowly center themselves before the PID takes over.
