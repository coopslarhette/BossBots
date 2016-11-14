package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Stone Mao & Cooper LaRhette
 */

public abstract class MecanumOpMode extends OpMode{
    /**
     * Drives the motor on the mechanum frame
     *
     * Precondition: motor 1-4 are valid motors, side is either left or right.
     *
     * @param motor4 Motor on the top left
     * @param motor3 Motor on the top right
     * @param motor2 Motor on the bottom right
     * @param motor1 Motor on the bottom left
     */
    public void driveOneJoystick(int gamepad, String side, DcMotor motor4, DcMotor motor3, DcMotor motor2, DcMotor motor1){
        double angle, length;
        switch (gamepad){
            case 1:
                if(side.equalsIgnoreCase("left")){
                    angle = this.getJoystickAngle(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    length = this.getDistance(gamepad1.left_stick_x, gamepad1.left_stick_y);
                } else {
                    angle = this.getJoystickAngle(gamepad1.right_stick_x, gamepad1.left_stick_y);
                    length = this.getDistance(gamepad1.right_stick_x, gamepad1.left_stick_y);
                }
                break;
            case 2:
                if(side.equalsIgnoreCase("left")){
                    angle = this.getJoystickAngle(gamepad2.left_stick_x, gamepad2.left_stick_y);
                    length = this.getDistance(gamepad2.left_stick_x, gamepad2.left_stick_y);
                } else {
                    angle = this.getJoystickAngle(gamepad2.right_stick_x, gamepad2.left_stick_y);
                    length = this.getDistance(gamepad2.right_stick_x, gamepad2.left_stick_y);
                }
                break;
            default:
                if(side.equalsIgnoreCase("left")){
                    angle = this.getJoystickAngle(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    length = this.getDistance(gamepad1.left_stick_x, gamepad1.left_stick_y);
                } else {
                    angle = this.getJoystickAngle(gamepad1.right_stick_x, gamepad1.left_stick_y);
                    length = this.getDistance(gamepad1.right_stick_x, gamepad1.left_stick_y);
                }
                break;
        }
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
    public double getJoystickAngle(double x, double y) {
        //First Figure out the Quadrant then find the angle
        if (-y >= 0) {
            //telemetry.addData("y>=0", Math.atan2(-y, x));
            //telemetry.addData("y<0", false);
            return Math.atan2(-y, x);
        } else if (-y < 0) {
            //telemetry.addData("y>=0", false);
            //telemetry.addData("y<0", Math.PI * 2 + Math.atan2(-y, x));
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
