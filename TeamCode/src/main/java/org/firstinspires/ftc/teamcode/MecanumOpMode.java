package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by stonemao on 11/11/2016.
 */

public abstract class MecanumOpMode extends OpMode{
    /**
     * Drives the motor on the mechanum frame
     *
     * @param motor4 Motor on the top left
     * @param motor3 Motor on the top right
     * @param motor2 Motor on the bottom right
     * @param motor1 Motor on the bottom left
     */
    public void driveJoystick(DcMotor motor4, DcMotor motor3, DcMotor motor2, DcMotor motor1){
        double angle = this.getAngle(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double length = this.getDistance(gamepad1.left_stick_x, gamepad1.left_stick_y);
        motor1.setPower(length * Math.sin(angle - Math.PI / 4));
        motor2.setPower(length * Math.cos(angle - Math.PI / 4));
        motor3.setPower(length * Math.sin(angle - Math.PI / 4));
        motor4.setPower(length * Math.cos(angle - Math.PI / 4));
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
            telemetry.addData("y<0", Math.PI * 2 + Math.atan2(-y, x));
            return 2 * Math.PI + Math.atan2(-y, x);
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
