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
 *Tests the different motors on the robot to get them configured correctly
 */

public class Mecanumwheel_Test extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor shooter;
    private DcMotor lift;

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
        shooter = hardwareMap.dcMotor.get("shooter");
        lift = hardwareMap.dcMotor.get("lift");

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
        //ROBOT BUTTONS CONTROL

        if (gamepad1.x) {
            motor1.setPower(1);
        } else if (gamepad1.y) {
            motor2.setPower(1);
        } else if (gamepad1.a) {
            motor3.setPower(1);
        } else if (gamepad1.b) {
            motor4.setPower(1);
        }
        lift.setPower(gamepad1.left_trigger);
        shooter.setPower(gamepad1.right_trigger);

        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Left Stick x: ", gamepad1.left_stick_x);
        telemetry.addData("Left Stick y: ", gamepad1.left_stick_y);
        telemetry.addData("Flywheel trigger: ", gamepad1.right_trigger);
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}



