## TeamCode Module

Welcome!

Team 524, The Boss Bot's repository

## Contains:

* MecanumOpMode
An abstract class which contains the code for holometric drive (Mecanum and Omniwheel). Currently
has only the joystick controls. Will be adding more codes at driving at certain angles.

## API

* ```java
   public void driveJoystick(DcMotor motor4, DcMotor motor3, DcMotor motor2, DcMotor motor1)`
  ```
Takes in four motors in a clockwise order starting from the top-left corner:

```
  *Omni-wheel*             *Mecanum Wheel*

  **[Front]**               **[Front]**

motor4  motor3             motor4  motor3

   /-----\                   []-----[]

    |   |                     |     |

    |   |                     |     |

   \-----/                   []-----[]

motor1  motor2             motor1  motor2
```