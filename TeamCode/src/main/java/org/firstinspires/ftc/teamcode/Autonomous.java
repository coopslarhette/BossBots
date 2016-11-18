/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Template: Linear OpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class Autonomous extends LinearOpMode implements SensorEventListener{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    public SensorManager sensorService;
    private Sensor magnetometer;
    private Sensor accelerometer;
    public float compassX;
    public float compassY;
    public float compassZ;
    public float accX;
    public float accY;
    public float accZ;

    // NEEDED FOR PID
    private double setx, sety, seth; //set points for x, y and theta
    private double lerrx, lerry, lerth; //last errors in x, y and theta
    private double errx, erry, erth; //errors in x, y and theta
    private double curx, cury, cuth; //current x, y and theta from IMU
    private double outx, outy, outh; //the output velocity values for x, y and theta
    private double derx, dery, dert; //derivatives of x, y and t(heta)
    //private double ierx, iery, iert; //integrals of x, y and t(heta) (not needed)
    //private double serx, sery, sert; //sums of the value-time products (ont needed)
    private double px, dx, py, dy, pt, dt; //the proportional, integral(not needed, so removed) and differential constants for x, y and t(heta)
    private long timeFromStart;//get the timer class thingy
    private long currTime; //current time
    private int interval; //interval for integration; should be the same as the sample period for the IMU
    private double[] forward; //the vector for taking me forward
    private double[] right; //the vector to take me right
    private double[] ccw; //the vector to take me counterclockwise
    private double[] resultant; //the resultant vector

    private double[] B;
    private double[] g;
    private double[] c;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor2 = hardwareMap.dcMotor.get("motor2");

        motor3 = hardwareMap.dcMotor.get("motor3");

        motor4 = hardwareMap.dcMotor.get("motor4");
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);

        sensorService = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        //Magnetometer initialization
        magnetometer = sensorService.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        //Accelerometer initialization
        accelerometer = sensorService.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        sensorService.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        sensorService.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_FASTEST);

        //x,y,z
        B = new double[3];
        g = new double[3];
        c = new double[3];

        waitForStart();
        runtime.reset();
        while(opModeIsActive()){
            telemetry.addData("B", "x: "+B[0]+" y: "+B[1]+" z: "+B[2]);
            telemetry.addData("Accelerometer", "x: "+accX+" y: "+accY+" z: "+accZ);
        }

    }
    public double[] resultant(double mySetx, double mySety, double mySeth) {
        setx = mySetx;
        sety = mySety;
        seth = mySeth;


        //get current variables from sensors
        //curx = ;
        //cury =;
        //cuth =;


        //errors in the robot's state are calculated as (setpoint-current)
        errx = setx - curx;
        erry = sety - cury;
        erth = seth - cuth;


        //the derivatives
        derx = (errx - lerrx) / interval;
        dery = (erry - lerry) / interval;
        dert = (erth - lerth) / interval;


      /* //the integrals
       ierx = (errx * interval);
       serx = serx + ierx;
       iery = (erry * interval);
       sery = sery + iery;
       iert = (erth * interval);
       sert = sert + iert;
       */


        // outputs for the variables
        outx = (px * errx) + (dx * derx);
        outy = (py * erry) + (dy * dery);
        outh = (pt * erth) + (dt * dert);


        //limit the outputs to [-1,1]
        if (outx >= 1) {
            outx = 1;
        }


        if (outy >= 1) {
            outy = 1;
        }


        if (outh >= 1) {
            outh = 1;
        }


        if (outx <= -1) {
            outx = -1;
        }


        if (outy <= -1) {
            outy = -1;
        }


        if (outh <= -1) {
            outh = -1;
        }


        //assign the vectors
        double forward[] = {(outx * -1), (outx * 1), (outx * 1), (outx * -1)};
        double right[] = {(outy * 1), (outy * 1), (outy * -1), (outy * -1)};
        double ccw[] = {(outh * 1), (outh * 1), (outh * 1), (outh * 1)};
        //double resultant[] = add(forward, right, ccw);


        //current error becomes last error
        lerrx = errx;
        lerry = erry;
        lerth = erth;
        return new double[3];

    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Ignoring this for now

    }
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        switch(sensorEvent.sensor.getType()){
            case Sensor.TYPE_ACCELEROMETER:
                accX = sensorEvent.values[0];
                accY = sensorEvent.values[1];
                accZ = sensorEvent.values[2];
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                compassX = sensorEvent.values[0];
                compassY = sensorEvent.values[1];
                compassZ = sensorEvent.values[2];
                break;
        }

        // 0 - X, 1 - Y, 2 - Z
        B[0]=compassX;
        B[1]=compassY;
        B[2]=compassZ;
    }
}
