package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Template: Iterative OpMode", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
@Disabled

/*

OBJECTIVE:
- Covert omni control to mecanum control

 */

public class Mecanumwheel extends OpMode { // Copied from Omniwheel
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor flywheel1;
    private DcMotor flywheel2;

    public Mecanumwheel(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motor3 = motor3;
        this.motor4 = motor4;
        this.flywheel1 = flywheel1;
        this.flywheel2 = flywheel2;

    }

    /**
     * Set all motor powers to 0
     */
    public void killMotors() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() { // Initialize
        telemetry.addData("Status", "Initialized");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        //flywheel1 = hardwareMap.dcMotor.get("flywheel");
        //flywheel2 = hardwareMap.dcMotor.get("flywheel2");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

        /*
         * Code to run ONCE when the driver hits PLAY
         */

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
    }

    public void motorPower() { // Basically public void loop(){}
        killMotors();
        //ROBOT BUTTONS CONTROL

        // Forwards and Backwards
        while (gamepad1.left_stick_y < 0) {
            motor4.setPower(-1);
            motor1.setPower(-1);
            motor3.setPower(1);
            motor2.setPower(1);
        }

        while (gamepad1.left_stick_y > 0) {
            motor4.setPower(1);
            motor1.setPower(1);
            motor3.setPower(-1);
            motor2.setPower(-1);
        }

        // Turning
        while (gamepad1.left_stick_x < 0) { //Right
            motor4.setPower(-1);
            motor1.setPower(-1);
            motor3.setPower(-1);
            motor2.setPower(-1);
        }
        while (gamepad1.left_stick_x > 0) { //Left
            motor4.setPower(1);
            motor1.setPower(1);
            motor3.setPower(1);
            motor2.setPower(1);
        }
/*
            // Flywheel
        if(gamepad1.right_trigger > 0){
           flywheel1.setPower(gamepad1.right_trigger);
            flywheel2.setPower(-gamepad1.right_trigger);
        } else{
            flywheel1.setPower(0);
            flywheel2.setPower(0);
        }
*/
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Left Stick x: ", gamepad1.left_stick_x);
        telemetry.addData("Left Stick y: ", gamepad1.left_stick_y);
        telemetry.addData("Flywheel trigger: ", gamepad1.right_trigger);
        // driveTrain(gamepad1.left_stick_x,gamepad1.left_stick_y);
        // Directions not exact
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    /*
    public void driveTrain(double x, double y){
        if (x < 0){ // Left
            motor1.setPower(1);
            motor2.setPower(-1);


        } else if (x > 0){ // Right
            motor1.setPower(-1);
            motor2.setPower(1);
        } else {
            motor1.setPower(0);
            motor2.setPower(0);
        }

        if (y < 0){ // Forward
            motor3.setPower(-1);
            motor4.setPower(1);


        } else if (y > 0){ // Backwards
            motor3.setPower(1);
            motor4.setPower(-1);


        } else {
            motor3.setPower(0);
            motor4.setPower(0);
        }
    }
        */
}


