package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MecanumWheel_Stone", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
@Disabled

public class Omniwheel extends OpMode { // Copied from TemplateOpMode_Iterative
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor springMotor;
    private DcMotor bBallMotor;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() { // Initialize
        telemetry.addData("Status", "Initialized");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);
        //springMotor = hardwareMap.dcMotor.get("springMotor");
        //bBallMotor = hardwareMap.dcMotor.get("bBallMotor");

        /* Structure of the robot
                (Front)                 Joystick Positions
          Motor 4      Motor 3                  -1
                /-----\                         |
                 |   |                    -1 ------- 1
                 |   |                          |
                \-----/                         1
          Motor 1     Motor 2
                (Back)
        */
        //flywheel1 = hardwareMap.dcMotor.get("flywheel");
        //flywheel2 = hardwareMap.dcMotor.get("flywheel2");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double angle = this.getAngle(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double length = this.getDistance(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //Different length at the same angle will change the motor powers
        /*
            ie. Joystick at 4 unit away from center generates 2 times the power of the joystick 2 units away
        */
        telemetry.addData("Cos", length * Math.cos(angle - Math.PI / 4));
        telemetry.addData("Sin", length * Math.sin(angle - Math.PI / 4));
        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y not negative", gamepad1.left_stick_y);
        if (Math.abs(gamepad1.right_stick_x) > .05) {
            motor1.setPower(gamepad1.right_stick_x);
            motor2.setPower(-gamepad1.right_stick_x);
            motor3.setPower(-gamepad1.right_stick_x);
            motor4.setPower(gamepad1.right_stick_x);
        } else {
            motor1.setPower(length * Math.sin(angle - Math.PI / 4));
            motor2.setPower(length * Math.cos(angle - Math.PI / 4));
            motor3.setPower(length * Math.sin(angle - Math.PI / 4));
            motor4.setPower(length * Math.cos(angle - Math.PI / 4));
        }
        //bBallMotor.setPower(gamepad1.right_trigger);
//        while (gamepad1.a) {
//            springMotor.setPower(1);
//        }
    }

    /**
     * Calculate the angle which the joystick is currently at
     *
     * @param x the x position of the joystick
     * @param y the y position of the joystick
     */
    public double getAngle(double x, double y) {
        //First Figure out the Quadrant then find the angle
        if (-y >= 0) {
            telemetry.addData("y>=0", Math.atan2(-y, x));
            telemetry.addData("y<0", false);
            return Math.atan2(-y, x);
        } else if (-y < 0) {
            telemetry.addData("y>=0", false);
            telemetry.addData("y<0", Math.PI * 2 - Math.atan2(-y, x));
            return Math.PI * 2 - Math.atan2(-y, x);
        }
        return 0;
    }

    /**
     * Calculate the distance from the center to where the joystic is currently at
     *
     * @param x the x position of the joystick
     * @param y the y position of the joystick
     */
    public double getDistance(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

}


