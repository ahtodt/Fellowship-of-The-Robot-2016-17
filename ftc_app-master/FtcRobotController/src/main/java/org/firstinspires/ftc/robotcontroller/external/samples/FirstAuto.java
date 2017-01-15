
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
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.locks.Lock;

import static java.lang.Math.*;




public class FirstAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //DcMotor leftMotor = null;
    //DcMotor rightMotor = null;
   I2cAddr leftColorI2c = I2cAddr.create8bit(0x4c);
    I2cAddr floorI2c = I2cAddr.create8bit(0x70);
    I2cAddr rightColorI2c = I2cAddr.create8bit(0x3c);
    I2cAddr frontRangeI2c = I2cAddr.create8bit(0x44);
    I2cAddr rightRangeI2c = I2cAddr.create8bit(0x34);
    I2cAddr leftRangeI2c= I2cAddr.create8bit(0x28);
    double left;
    double right;
    double turnTolerance = 1;
    double newHeading;
    double oneRotation =420;
    double currentHeading;
    double driveGain = .007;
    double directionAdjust;
    double startHeading;
    double headingError;
    double driveSteering;
    double leftPower;
    double rightPower;
    double engagePower =.2;
    double firingSpeed = .9;
    double cockingSpeed = .5;
    int mortarFreeState = 1290;
    int mortarReadyState = 1290;
    int mortarEngagedState = 300;
    int driveDistance = (560);
    DcMotor left_drive1;
    DcMotor right_drive1;
    DcMotor left_drive2;
    DcMotor right_drive2;
    DcMotor mortar;
    DcMotor cap_ball_lift;
    DcMotor cap_ball_tilt;
    DcMotor particle_collector;
    Servo left_beacon;
    Servo right_beacon;
    Servo collector_gate;
    Servo magazine_cam;
    Servo mortar_gate;
    GyroSensor gyro;
    ModernRoboticsI2cRangeSensor front_range;
    ModernRoboticsI2cRangeSensor right_range;
    ModernRoboticsI2cRangeSensor left_range;
    ColorSensor floor_seeker;
    ColorSensor left_color;
    ColorSensor right_color;
    public void stopMotors(){
        right_drive1.setPower(0);
        left_drive1.setPower(0);
        right_drive2.setPower(0);
        left_drive2.setPower(0);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status first", "Initialized");
        telemetry.update();
        left_color = hardwareMap.colorSensor.get("left_color");
        right_color = hardwareMap.colorSensor.get("right_color");
        floor_seeker = hardwareMap.colorSensor.get("floor_seeker");
        left_color.setI2cAddress(leftColorI2c);
        right_color.setI2cAddress(rightColorI2c);
        floor_seeker.setI2cAddress(floorI2c);
        floor_seeker.enableLed(false);
        floor_seeker.enableLed(true);
        right_color.enableLed(false);
        left_color.enableLed(false);
        right_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "right_range");
        left_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_range");
        front_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front_range");
        front_range.setI2cAddress(frontRangeI2c);
        left_range.setI2cAddress(leftRangeI2c);
        right_range.setI2cAddress(rightRangeI2c);
        left_drive1 = hardwareMap.dcMotor.get("left_drive1");
        left_drive2 = hardwareMap.dcMotor.get("left_drive2"); 
        right_drive1 = hardwareMap.dcMotor.get("right_drive1");
        right_drive2 = hardwareMap.dcMotor.get("right_drive2");
        right_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive2.setDirection(DcMotorSimple.Direction.REVERSE);
        mortar = hardwareMap.dcMotor.get("mortar");
        mortar.setDirection(DcMotorSimple.Direction.REVERSE);
        mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mortar_gate = hardwareMap.servo.get("mortar_gate");
        cap_ball_lift = hardwareMap.dcMotor.get("cap_ball_lift");
        cap_ball_tilt = hardwareMap.dcMotor.get("cap_ball_tilt");
        left_beacon = hardwareMap.servo.get("left_beacon");
        right_beacon = hardwareMap.servo.get("right_beacon");
        particle_collector = hardwareMap.dcMotor.get("particle_collector");
        collector_gate = hardwareMap.servo.get("collector_gate");
        magazine_cam = hardwareMap.servo.get("magazine_cam");
        gyro = hardwareMap.gyroSensor.get("gyro");

        /*gyro.calibrate();
        while(gyro.isCalibrating()){
            sleep(40);
        }*/

        waitForStart();
        runtime.reset();

        while(true&&opModeIsActive()){
            telemetry.addData("right", right_range.cmUltrasonic());
            telemetry.update();
        }
        //positionToShoot();
        //shootBall();
        //driveStraight();
        //driveToWall();
        //gyroTurn(-90);
        //findWhiteLine();
        //wallSense();
        //shoot(30);
        right_drive1.setPower(0);
        right_drive2.setPower(0);
        left_drive1.setPower(0);
        left_drive2.setPower(0);

        }

        /*public void stopMotors(){
            right_drive1.setPower(0);
            left_drive1.setPower(0);
            right_drive2.setPower(0);
            left_drive2.setPower(0);
        }
        public void setPowerLeft(double power){
            left_drive1.setPower(power);
            left_drive2.setPower(power);
        }
        public void setPowerRight(double power){
            right_drive1.setPower(power);
            right_drive2.setPower(power);
        }
        public void positionToShoot(){
            right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while(right_drive1.getCurrentPosition()<driveDistance&&left_drive2.getCurrentPosition()<driveDistance) {

                setPowerLeft(.05);
                setPowerRight(.05);
            }
            stopMotors();
        } */

        /*public void shootBall(){
            mortar.setPower(firingSpeed);
            mortar.setTargetPosition(mortarFreeState);
            sleep(1000);
            mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mortar.setPower(cockingSpeed);
            mortar.setTargetPosition(mortarEngagedState);

        }*/

        /*public void shoot(int shots){
            shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            for(int i=0;i<shots;i++){
             shooterMotor.setPower(.3);
                shooterMotor.setTargetPosition(420);
                shooterMotor.setPower(0);

                shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(10);
            }
        }*/

         public void findWhiteLine(){
             while(right_range.cmUltrasonic()>30){
                 left_drive1.setPower(0.1);
                 left_drive2.setPower(0.1);
                 right_drive1.setPower(0.1);
                 right_drive2.setPower(0.1);
                 //wallSense();
             }stopMotors();
                //redBeaconPress();

        }

        /*public void wallSense(){
            while(right_range.cmUltrasonic()<20){
                right_drive1.setPower(.01);
                right_drive2.setPower(.01);
                left_drive1.setPower(.01);
                left_drive2.setPower(.01);
            }while(right_range.cmUltrasonic()>=20){
                left_drive1.setPower(.01);
                left_drive2.setPower(.01);
                right_drive1.setPower(.01);
                right_drive2.setPower(.01);
            }
        }*/

        //right_drive1.setPower(0);
        //right_drive2.setPower(0);
        //left_drive1.setPower(0);
        //left_drive2.setPower(0);

    }

        /*public void redBeaconPress(){
            right_drive1.setMaxSpeed(0);
            right_drive2.setMaxSpeed(0);
            left_drive1.setMaxSpeed(0);
            left_drive2.setMaxSpeed(0);
            if (right_color.red() > 1.5) {
                right_beacon.setPosition(.15);
                //continueForward();
            }
            else if (right_color.blue() > 1.5) {
                //add code for specific position/encoder ticks forward
                right_beacon.setPosition(.15);
                //continueForward();
            }
    */

        /*public void continueForward(){
            if(floor_seeker.red()<5){
                wallSense();
            }
            else if (floor_seeker.red()>5){
                secondBeaconPress();
            }
        }*/

    /* public void secondBeaconPress(){
            right_drive1.setMaxSpeed(0);
            right_drive2.setMaxSpeed(0);
            left_drive1.setMaxSpeed(0);
            left_drive2.setMaxSpeed(0);
            if (right_color.red() > 1.5) {
                right_beacon.setPosition(.15);
                //park();
            }
            else if (right_color.blue() > 1.5) {
                //add code for specific position/encoder ticks forward
                right_beacon.setPosition(.15);
                //park();
            }
        }*/

        /*public void park(){
            //enter some code about how to park
        }*/
