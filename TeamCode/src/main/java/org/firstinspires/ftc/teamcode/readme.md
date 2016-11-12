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

 Parameters | Explanation
 --- | ---
 `int gamepad` | which gamepad to use: 1 or 2. Default: 1
 `String side` | Which joystick to use on gamepad: right or left. Default: right
 `DcMotor motor4` | Motor on the top-left corner (refer to diagram above)
 `DcMotor motor3` | Motor on the top-right aorner (refer to diagram above)
 `DcMotor motor2` | Motor on the bottom-right corner (refer to diagram above)
 `DcMotor motor1` | Motor on the bottom-left corner (refer to diagram above)
