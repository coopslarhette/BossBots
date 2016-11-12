## TeamCode Module

Welcome!

Team 524, The Boss Bot's repository

## Contains:

* MecanumOpMode
An abstract class which contains the code for holometric drive (Mecanum and Omniwheel). Currently
has only the joystick controls. Will be adding more codes at driving at certain angles.

## Structure of robot

```
  Omni-wheel               Mecanum Wheel
   [Front]                    [Front]
motor4  motor3             motor4  motor3
   /-----\                   []-----[]
    |   |                     |     |
    |   |                     |     |
   \-----/                   []-----[]
motor1  motor2             motor1  motor2
```

# API

* `public void driveOneJoystick(int gamepad, String side, DcMotor motor4, DcMotor motor3, DcMotor motor2, DcMotor motor1)`
 * **Function** Drives the motor according to gamepad1 left or right stick position.
 * **Parameters:**
  *Which Gamepad to use: gamepad1 or gamepad2, will default to gamepad1
  *Which joystick of the gamepad to use: left or right (will default to right)
  *Takes in four motors in a clockwise order starting from the top-left corner, refer to figure above

