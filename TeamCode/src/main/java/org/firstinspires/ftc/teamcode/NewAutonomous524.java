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

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "Team 524 Autonomous", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
@Disabled
public class NewAutonomous524 extends MecanumOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor belt;
    private DcMotor eightyTwenty;
    private DcMotor sweeper;



    //Phone sensors
    private Sensor magnetometer;
    private Sensor accelerometer;

    private String teamColor;
    private DcMotor shooter;
    private Servo ballKeeper;
    private Servo flicker;

    //PID variables
    private double[] acc, vel, pos, setpos, errpos, output; //acceleration (WITHOUT g), velocity, position, setpoint (position), error in position, output (scaled voltage)
    private double[] kp, kd; //output (scaled voltage)


    /*
    *   Motor position
    *
    * motor4     motor3
    *    []-------[]
    *      |     |
    *      |     |
    *      |     |
    *    []-------[]
    *  motor1    motor2
    */

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        acc = new double[3];
        vel = new double[3];
        pos = new double[3];
        setpos = new double[3];
        errpos = new double[3];
        output = new double[3];

        //set initial positions; ask Sagnick for details of what to do
        pos[0] = ;
        pos[1] = ;
        pos[2] = ;

        //set the proportional and derivative constants
        kp[0] = ;
        kp[1] = ;
        kp[2] = ;
        kd[0] = ;
        kd[1] = ;
        kd[2] = ;

        light = hardwareMap.lightSensor.get("light");

        color = hardwareMap.colorSensor.get("color");

        teamColor = "r";

        //Initialize sensor service
        sensorService = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        telemetry.addData("Status", "Created SensorService");
        //Magnetometer initialization
        magnetometer = sensorService.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        telemetry.addData("Status", "Created Magnenetometer");
        //Accelerometer initialization
        accelerometer = sensorService.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        telemetry.addData("Status", "Created Accelerometer");
        sensorService.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME);
        telemetry.addData("Status", "Registered acclerometer");
        sensorService.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_GAME);
        telemetry.addData("Status", "Registered magnetometer");


        shooter = hardwareMap.dcMotor.get("shooter");

        ballKeeper = hardwareMap.servo.get("ballKeeper");
        flicker = hardwareMap.servo.get("flicker");
        flicker.setPosition(0.55);
        ballKeeper.setPosition(0.0);
        telemetry.addData("Status", "Initialized");
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
        //do something about timing

        //sensor stuff here



        //this is the PD controller
        public double[] output(setpos[0], setpos[1]) {
            for (int i = 0; i <= 2; i++) {
                output[i] = (kp[i] * (setpos[i] - pos[i])) - (kd[i] * vel[i]);
            }

            return output; //why is this giving an error?
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}