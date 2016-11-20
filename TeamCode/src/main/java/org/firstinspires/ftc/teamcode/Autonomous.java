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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.StringBuilderPrinter;

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
    public ColorSensor color;
    public float compassX;
    public float compassY;
    public float compassZ;
    public float accX;
    public float accY;
    public float accZ;

    // NEEDED FOR PID
    private double[] setx, sety; //set points for x, y and theta
    private double seth; //set points for x, y and theta
    private double lerrx, lerry, lerth; //last errors in x, y and theta
    private double errx, erry, erth; //errors in x, y and theta
    private double curx, cury, cuth; //current x, y and theta from IMU (pun on cury!)
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

    private double[] b; //initial magnetic field vector
    private double[] bprime; //projection of magnetic field on the plane perpendicular to the gravitational field vector
    private double[][] matrixT; //transformation matrix that takes in vectors in the gamefield reference frame and outputs vectors in the phone reference frame
    private double[] g; // initial gravitational accl vector
    private double[] c; // I don't know what this is.
    private double[] gprime; //unit vector in the direction of g
    private double[] x; //cross product of y and g
    private double[] y; // unit vector in the direction of bprime
    private double[] h; //runtime acceleration vector with gravity
    private double[] a; //runtime acceleration vector withOUT gravity
    private double[] s; //runtime displacement vector
    private double[] bcurr; //runtime magnetic field vector (pun!)
    private double[] setxT; //x setpoint in phone coordinate system
    private double[] setyT; //y setpoint in phone coordinate system
    private String teamColor;

    @Override
    public void runOpMode() {
        // Initialize Motor
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor2 = hardwareMap.dcMotor.get("motor2");

        motor3 = hardwareMap.dcMotor.get("motor3");

        motor4 = hardwareMap.dcMotor.get("motor4");
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize sensor service
        sensorService = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        //Magnetometer initialization
        magnetometer = sensorService.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        //Accelerometer initialization
        accelerometer = sensorService.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        //Color
        color = hardwareMap.colorSensor.get("color");
        //Adds both the sensors to the sensorService
        sensorService.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        sensorService.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_FASTEST);

        //x,y,z
        b = new double[3];
        g = new double[3];
        c = new double[3];
        h = new double[3];
        a = new double[3];
        gprime = new double[3];
        bcurr = new double[3];
        setxT = new double[3];
        setyT = new double[3];
        setx = new double[3];
        sety = new double[3];
        //Stores all acceleration of the phone
        g[0] = accX;
        g[1] = accY;
        g[2] = accZ;
        //Stores all unit vector value
        b[0] = compassX;
        b[1] = compassY;
        b[2] = compassZ;
        bprime=new  double[3];
        gprime=new  double[3];
        x = new  double[3];
        y = new  double[3];
        s = new  double[3];
        matrixT = new double[3][3];
        for (int i=0; i <= 2; i++) {
            gprime[i] = (euclidianNorm(g[i], g[0], g[1], g[2]));
        }

        for (int i = 0; i <= 2; i++) {
            bprime[i] = (b[i] - (b[i]*euclidianNorm((g[i]*b[i]), g[0], g[1], g[2])));
        }

        for (int i = 0; i <=2; i++) {
            y[i] = (euclidianNorm(bprime[i], bprime[0], bprime[1], bprime[2]));
        }

        x[0] = (y[1]*gprime[2]) - (y[2]*gprime[1]);
        x[1] = (y[0]*gprime[2]) - (y[2]*gprime[0]);
        x[1] = (y[0]*gprime[1]) - (y[1]*gprime[0]);

        for(int i = 0; i<3; i++){
            matrixT[i][0] = x[i];
            matrixT[i][1] = y[i];
            matrixT[i][2] = gprime[i];
        }

        //this should be runtime stuff
        h[0] = accX;
        h[1] = accY;
        h[2] = accZ;

        for (int i = 0; i <= 2; i++) {
            a[i] = h[i] - g[i];
        }


        waitForStart();
        runtime.reset();
        while(opModeIsActive()){
            telemetry.addData("Accelerometer", "x: "+accX+" y: "+accY+" z: "+accZ);
            telemetry.addData("Magnetic/Compass", "x: "+compassX+" y: "+compassY+" z: "+compassZ);
        }

    }
    public double[] resultant(double[] mySetx, double[] mySety, double mySeth) {
        setx = mySetx;
        sety = mySety;
        seth = mySeth;

        setxT[0] = matrixT[0][0]* ;

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

    //this is a mosnomer; you actually get a unit vector out of this
    public double euclidianNorm(double numerator, double a, double b, double c){
        return numerator/Math.sqrt(Math.pow(a,2)+Math.pow(b,2)+Math.pow(c,2));
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Ignoring this for now

    }
    public void rightColor() {
        //Red > blue --> red
        if (teamColor.equalsIgnoreCase("r")) {
            if (color.red() > color.blue()) {
                telemetry.addData("Color", "Red - YAY!!!");
            } else if (color.red() < color.blue()) {
                // Move to other beacon
                telemetry.addData("Color", "Blue");
            } else {
                telemetry.addData("Color", "Not any color!!!!!");
            }
        } else {
            if (color.red() < color.blue()) {
                telemetry.addData("Color", "Blue - YAY!!!");
            } else if (color.red() > color.blue()) {
                // Move to other beacon
                telemetry.addData("Color", "Red");
            } else {
                telemetry.addData("Color", "Not any color!!!!!");
            }
        }
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
        h[0] = accX;
        h[1] = accY;
        h[2] = accZ;

        for (int i = 0; i <= 2; i++) {
            a[i] = h[i] - g[i];
        }

        for (int i = 0; i <= 2; i++) {
            s[i] = (0.5*a[i]*interval*interval);
        }

        curx += s[0];
        cury += s[1];

        bcurr[0] = compassX;
        bcurr[1] = compassY;
        bcurr[2] = compassZ;

        cuth = dotProduct(bcurr, y);


    }

    public double dotProduct(double[] p, double[] q) {
        double product = (p[0]*q[0]) + (p[1]*q[1]) + (p[2]*q[2]);
        return product;
    }
}
