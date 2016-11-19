package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Stone Mao & Cooper LaRhette
 */

public abstract class MecanumOpMode extends OpMode implements SensorEventListener {

    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;


    private Servo buttonPresser;

    public String teamColor;

    public LightSensor light;
    private final double whiteLight = 0.1;
    private final double deviation = 0.1;

    //public double startingAngle;

    public ColorSensor color;

    public SensorManager sensorService;
    private float compassX;
    private float compassY;
    private float compassZ;

    public void turning() {
        motor1.setPower(-gamepad1.left_stick_x);
        motor2.setPower(gamepad1.left_stick_x);
        motor3.setPower(gamepad1.left_stick_x);
        motor4.setPower(-gamepad1.left_stick_x);
    }

    /**
     * Drive the holonomic drivetrain with one joystick
     *
     * @param gamepad Which gamepad to use: 1 or 2. Default 1
     * @param side    Which joystick to use: "left" or "right." Default "right"
     */
    public void driveOneJoystick(int gamepad, String side) {
        double angle, length;
        switch (gamepad) {
            case 1:
                if (side.equalsIgnoreCase("left")) {
                    angle = this.getJoystickAngle(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    length = this.getDistance(gamepad1.left_stick_x, gamepad1.left_stick_y);
                } else {
                    angle = this.getJoystickAngle(gamepad1.right_stick_x, gamepad1.left_stick_y);
                    length = this.getDistance(gamepad1.right_stick_x, gamepad1.left_stick_y);
                }
                break;
            case 2:
                if (side.equalsIgnoreCase("left")) {
                    angle = this.getJoystickAngle(gamepad2.left_stick_x, gamepad2.left_stick_y);
                    length = this.getDistance(gamepad2.left_stick_x, gamepad2.left_stick_y);
                } else {
                    angle = this.getJoystickAngle(gamepad2.right_stick_x, gamepad2.left_stick_y);
                    length = this.getDistance(gamepad2.right_stick_x, gamepad2.left_stick_y);
                }
                break;
            default:
                if (side.equalsIgnoreCase("left")) {
                    angle = this.getJoystickAngle(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    length = this.getDistance(gamepad1.left_stick_x, gamepad1.left_stick_y);
                } else {
                    angle = this.getJoystickAngle(gamepad1.right_stick_x, gamepad1.left_stick_y);
                    length = this.getDistance(gamepad1.right_stick_x, gamepad1.left_stick_y);
                }
                break;
        }
        //Calculates the motor power based off of trignometric functions
        double sin2and4 = length * Math.round(Math.sin(angle - Math.PI / 4)*10.0)/10.0;
        double cos1and3 = length * Math.round(Math.cos(angle - Math.PI / 4)*10.0)/10.0;

        //Driving
        if (Math.abs(gamepad1.right_stick_x) > 0.03) {
            turning();
        } else {
            motor1.setPower(cos1and3);
            motor2.setPower(sin2and4);
            motor3.setPower(cos1and3);
            motor4.setPower(sin2and4);
        }

        telemetry.addData("sin", sin2and4);
        telemetry.addData("cos", cos1and3);

    }

    public void turn(int angle) {
//        telemetry.addData("x", compassX);
//        telemetry.addData("y", compassY);
//        telemetry.addData("z", compassZ);



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
            return Math.atan2(-y, x);
        } else if (-y < 0) {
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

    public void lineTracking() {
        while (light.getLightDetected() <= whiteLight + deviation && light.getLightDetected() >= whiteLight - deviation) {
            //Forward method
        }
        turn(90);
        //Forward method
        //Move right of the beacon
    }

    public void rightColor() {
        //Red > blue --> red
        if (teamColor.equalsIgnoreCase("r")) {
            if (color.red() > color.blue()) {
                buttonPresser.setPosition(1);
            } else if (color.red() < color.blue()) {
                // Move to other beacon
                buttonPresser.setPosition(1);
            } else {
                telemetry.addData("Error!!!!", "Not any color!!!!!");
            }
        } else {
            if (color.red() < color.blue()) {
                buttonPresser.setPosition(1);
            } else if (color.red() > color.blue()) {
                // Move to other beacon
                buttonPresser.setPosition(1);
            } else {
                telemetry.addData("Error!!!!", "Not any color!!!!!");
            }
        }
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Ignoring this for now

    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        compassX = sensorEvent.values[0];
        compassY = sensorEvent.values[1];
        compassZ = sensorEvent.values[2];
    }
}
