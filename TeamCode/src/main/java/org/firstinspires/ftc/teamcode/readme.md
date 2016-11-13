## TeamCode Module

Welcome!

Team 524, The Boss Bot's repository

## Contains:

* MecanumOpMode
An abstract class which contains the code for holonomic drive (Mecanum and Omniwheel). Currently
has only the joystick controls. Will be adding more codes for driving at certain angles.

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

## Joystick Coordinate/ Position (Hardware Configuration)

```
Starting Position

   [Y-Axis]
     -1
      |
-1 -- o -- 1 [X-axis]
      |
      1

Example Position

   [Y-Axis]
     -1
      | o (0.74, -0.74)
-1 --   -- 1 [X-axis]
      |
      1
```

# API

* `public void driveOneJoystick(int gamepad, String side, DcMotor motor4, DcMotor motor3, DcMotor motor2, DcMotor motor1)`
 * **Function** Drives the motor according to gamepad1 left or right stick position.
 * **Parameters:**

 Parameters | Explanation
 --- | ---
 `int gamepad` | which gamepad to use: 1 or 2. Default: 1.
 `String side` | Which joystick to use on gamepad: right or left. Default: right.
 `DcMotor motor4` | Motor on the top-left corner (refer to diagram above).
 `DcMotor motor3` | Motor on the top-right corner (refer to diagram above).
 `DcMotor motor2` | Motor on the bottom-right corner (refer to diagram above).
 `DcMotor motor1` | Motor on the bottom-left corner (refer to diagram above).

* `public double getJoystickAngle(double x, double y)`
 * **Function** Gets the angle (in radians) which the joystick is at currently relative to the center/resting point. Refer to joystick coordinates above.
 * **Parameters:**

 Parameters | Explanation
  --- | ---
  double x | x coordinate of gamepad
  double y | y coordinate of gamepad

* `public double getDistance(double x, double y)`
 * **Function** Gets the joystick's distance relative from the center/resting point. Refer to joystick coordinates above.
 * **Parameters:**

 Parameters | Explanation
  --- | ---
  double x | x coordinate of gamepad
  double y | y coordinate of gamepad