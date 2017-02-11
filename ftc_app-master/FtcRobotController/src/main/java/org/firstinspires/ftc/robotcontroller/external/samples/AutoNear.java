
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


//delay, longer drive before shoot, turn for square after shoot, only shoot one or fix the mortar

public class AutoNear extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //DcMotor leftMotor = null;
    //DcMotor rightMotor = null;
    I2cAddr leftColorI2c = I2cAddr.create8bit(0x4c);
    I2cAddr floorI2c = I2cAddr.create8bit(0x70);
    I2cAddr rightColorI2c = I2cAddr.create8bit(0x3c);
    I2cAddr leftRangeI2c = I2cAddr.create8bit(0x28);
    double left;
    double right;
    double turnTolerance;
    double newHeading;
    double oneRotation = 420;
    double currentHeading;
    double powerFloor;
    double floor = .015;
    double driveGain = .0003;
    int distance = 1450;
    double directionAdjust;
    double startHeading;
    double headingError;
    double driveSteering;
    double leftPower;
    double rightPower;
    double engagePower = .2;
    double firingSpeed = .9;
    double cockingSpeed = .5;
    double mortarGateUp = .6;
    double mortarGateDown = 1;
    double camUp = 0;
    double camMid = .3;
    double camDown = .5;
    double camZero = 1;
    double PCGateUp = .3;
    double PCGateDown = .75;
    double leftBeaconOut = .34;
    double rightBeaconOut = .41;
    double leftBeaconIn = .19;
    double rightBeaconIn = .26;
    int mortarFreeState = 1440;
    int mortarEngagedState = 300;
    int driveDistance = (525);

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
    int shots = 1;
    ModernRoboticsI2cRangeSensor left_range;
    ColorSensor floor_seeker;
    ColorSensor left_color;
    ColorSensor right_color;
    double target = 20;
    double rangeDistance = left_range.cmUltrasonic();
    double error = rangeDistance - target;
    double differential = error * 0.1;
    double rightVelocity = right_drive1.getPower();
    double leftVelocity = left_drive1.getPower();
    double leftVelocityCorrection = leftVelocity + differential;
    double rightVelocityCorrection = rightVelocity + differential;
    //int rightVelocityCorrectionInt = (int)Math.round(rightVelocityCorrection);
    //int leftVelocityCorrectionInt = (int)Math.round(leftVelocityCorrection);



    public void stopMotors() {
        right_drive1.setPower(0);
        left_drive1.setPower(0);
        right_drive2.setPower(0);
        left_drive2.setPower(0);
    }

    public void setPowerLeft(double power) {
        left_drive1.setPower(power);
        left_drive2.setPower(power);
    }

    public void setPowerRight(double power) {
        right_drive1.setPower(power);
        right_drive2.setPower(power);
    }

    public void positionToShoot() {
        right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        magazine_cam.setPosition(camUp);
        while (right_drive1.getCurrentPosition() < driveDistance && left_drive2.getCurrentPosition() < driveDistance&&opModeIsActive()) {

            setPowerLeft(.2);
            setPowerRight(.2);
        }
        stopMotors();
    }


    public void waitForTen() {
        while ((getRuntime() < 5) && opModeIsActive()) {
        }
    }

    public void shootBall() {
        mortar.setPower(firingSpeed);
        mortar.setTargetPosition(mortarFreeState);
        mortar_gate.setPosition(mortarGateDown);
        sleep(500);
        mortar_gate.setPosition(mortarGateUp);
        sleep(1000);
        mortar_gate.setPosition(mortarGateDown);
        mortar.setPower(firingSpeed);
        mortar.setTargetPosition(mortarFreeState * 2);
    }

    public void capBall() {
        while (right_drive1.getCurrentPosition() < (driveDistance * 2.25)
                && left_drive2.getCurrentPosition() < (driveDistance * 2.25)&&opModeIsActive()) {
            left_drive1.setPower(.1);
            left_drive2.setPower(.1);
            right_drive1.setPower(.2);
            right_drive2.setPower(.2);
           /* while(getRuntime() < 2 ) {
                left_drive1.setPower(-1);
                left_drive1.setMaxSpeed(-200);
                left_drive2.setPower(-1);
                left_drive2.setMaxSpeed(-200);
                right_drive1.setPower(0);
                right_drive2.setPower(0);
            }
            while(gyro.getHeading() > 190) {
                left_drive1.setPower(-1);
                left_drive1.setMaxSpeed(-200);
                left_drive2.setPower(-1);
                left_drive2.setMaxSpeed(-200);
                right_drive1.setPower(0);
                right_drive2.setPower(0);
            }*/
        }
        stopMotors();

    }


    public void findWhiteLine() {
        right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (floor_seeker.green() < 6&&opModeIsActive()) {
            left_drive1.setPower(0.1);
            left_drive2.setPower(0.1);
            right_drive1.setPower(0.1);
            right_drive2.setPower(0.1);
            telemetry.addData("sensor", floor_seeker.blue());
            //wallSense();
        }
        stopMotors();

    }
    public void gyroTurnLeft(double desiredAngle) {
        do {

            if (gyro.getHeading() > 180) {
                currentHeading = gyro.getHeading() - 360;
            }else{
                currentHeading = gyro.getHeading();
            }
        headingError = desiredAngle - currentHeading;
            if(headingError<0){
                powerFloor = -floor;
            }else{
                powerFloor = floor;
            }
        driveSteering = (headingError * driveGain)+powerFloor;

        leftPower = driveSteering;
        if(leftPower>.2){
            leftPower = .2;
        }else if(leftPower<-.2){
            leftPower = -.2;
        }

        rightPower = -driveSteering;
        if(rightPower>.2){
            rightPower = .2;
        }else if(rightPower < -.2) {
            rightPower = -.2;
        }

        setPowerLeft(leftPower);
        setPowerRight(rightPower);

        telemetry.addData("in turn loop", leftPower);
            telemetry.addData("gyro", gyro.getHeading());

    } while (Math.abs(currentHeading-desiredAngle)>turnTolerance&&opModeIsActive());

    stopMotors();
}
    public void wallTurn(int target){
        stopMotors();
        right_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerLeft(.1);
        setPowerRight(.1);
        left_drive1.setTargetPosition(-target);
        left_drive2.setTargetPosition(-target);
        right_drive1.setTargetPosition(target);
        right_drive2.setTargetPosition(target);
            while (left_drive1.getCurrentPosition() > -target && opModeIsActive()) {


        }
    }
    public void lineTurn(int angle){
        stopMotors();
        right_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerLeft(.1);
        setPowerRight(.1);
        left_drive1.setTargetPosition(-angle+20);
        left_drive2.setTargetPosition(-angle+20);
        right_drive1.setTargetPosition(angle+10);
        right_drive2.setTargetPosition(angle+10);
        while (left_drive1.getCurrentPosition() < (-angle+20) && opModeIsActive()) {


        }
    }

        public void wallSense() {
            while (left_range.cmUltrasonic() < 20) {
                right_drive1.setPower(.01);
                right_drive2.setPower(.01);
                left_drive1.setPower(.01);
                left_drive2.setPower(.01);
            }
            while (left_range.cmUltrasonic() >= 20) {
                left_drive1.setPower(.01);
                left_drive2.setPower(.01);
                right_drive1.setPower(.01);
                right_drive2.setPower(.01);
            }
            stopMotors();
        }
            /* right_drive1.setPower(.2);
               right_drive2.setPower(.2);
               left_drive1.setPower(.2);
               left_drive1.setPower(.2);*/
    /*         right_drive1.setMaxSpeed(1680);
               right_drive2.setMaxSpeed(1680);
               left_drive1.setMaxSpeed(1680);
               left_drive2.setMaxSpeed(1680);*/
    /*      if(distance != target){
               right_drive1.setPower(rightVelocityCorrection); //potentially reverse different motors in hardware map
               right_drive2.setPower(rightVelocityCorrection);
               left_drive1.setPower(leftVelocityCorrection);
               left_drive2.setPower(leftVelocityCorrection);*/
                //stopMotors();



        public void firstBeaconPress() {
            stopMotors();
            if (left_color.red() > 1.5) {
                left_beacon.setPosition(leftBeaconOut);
               smallMove1();
                sleep(500);
                left_beacon.setPosition(leftBeaconIn);
                smallMove2();
            } else if (left_color.blue() > 1.5) {
                //add code for specific position/encoder ticks forward
                smallMove1();
                sleep(500);
                left_beacon.setPosition(leftBeaconOut);
                smallMove2();
                sleep(500);
                left_beacon.setPosition(leftBeaconIn);
            }else{
                forward();
            }
            stopMotors();
        }
    public void smallMove1(){
        int encoderStart1 = right_drive1.getCurrentPosition();
        while(right_drive1.getCurrentPosition()<encoderStart1+70&&opModeIsActive()){
            setPowerLeft(.05);
            setPowerRight(.05);
        }

    }
    public void smallMove2(){

        int encoderStart2 = right_drive1.getCurrentPosition();
        while(right_drive1.getCurrentPosition()<encoderStart2+70&&opModeIsActive()){
            setPowerLeft(.05);
            setPowerRight(.05);
        }

    }

    public void forward(){

        int encoderStart = right_drive1.getCurrentPosition();
        while(right_drive1.getCurrentPosition()<encoderStart+100&&opModeIsActive()){
            setPowerLeft(.1);
            setPowerRight(.1);
        }
    }
    public void turnToWhiteLine(double desiredAngle) {
        do {

            if (gyro.getHeading() > 180) {
                currentHeading = gyro.getHeading() - 360;
            }else{
                currentHeading = gyro.getHeading();
            }
            headingError = desiredAngle - currentHeading;
            if(headingError<0){
                powerFloor = -floor;
            }else{
                powerFloor = floor;
            }
            driveSteering = (headingError * driveGain)+powerFloor;

            leftPower = driveSteering;
            if(leftPower>.2){
                leftPower = .2;
            }else if(leftPower<-.2){
                leftPower = -.2;
            }

            rightPower = -driveSteering*.9;
            if(rightPower>.2){
                rightPower = .2;
            }else if(rightPower < -.2) {
                rightPower = -.2;
            }

            setPowerLeft(leftPower);
            setPowerRight(rightPower);

            telemetry.addData("in turn loop", leftPower);
            telemetry.addData("gyro", gyro.getHeading());

        } while (Math.abs(currentHeading-desiredAngle)>turnTolerance&&opModeIsActive());

        stopMotors();
    }
    public void driveStraight(){
        right_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(left_drive1.getCurrentPosition()<distance||left_drive2.getCurrentPosition()<distance||right_drive1.getCurrentPosition()<distance||right_drive2.getCurrentPosition()<distance&&opModeIsActive()){
            /*currentHeading = gyro.getHeading();
                    if(gyro.getHeading()>180){
                        currentHeading = gyro.getHeading()-360;
                    }
            if(currentHeading<-65){
                setPowerLeft(.9);
                setPowerRight(.9);
                left_drive1.setMaxSpeed(700);
                left_drive2.setMaxSpeed(700);
                right_drive1.setMaxSpeed(400);
                right_drive2.setMaxSpeed(400);

            }else if(currentHeading>-65){
                setPowerLeft(.9);
                setPowerRight(.9);
                left_drive1.setMaxSpeed(400);
                left_drive2.setMaxSpeed(400);
                right_drive1.setMaxSpeed(700);
                right_drive2.setMaxSpeed(700);
            }*/
            setPowerRight(.2);
            setPowerLeft(.2);
        }
        stopMotors();
    }
    public void turnToWall(){
        turnTolerance = 1;
        wallTurn(98);
        sleep(500);
    }
    public void turnNormal(){
        turnTolerance = 1;
        lineTurn(-98);
        sleep(500);
    }
    public void driveToWall(){
        right_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void secondBeaconPress(){
            right_drive1.setMaxSpeed(0);
            right_drive2.setMaxSpeed(0);
            left_drive1.setMaxSpeed(0);
            left_drive2.setMaxSpeed(0);
            if (left_color.red() > 1.5) {
                left_beacon.setPosition(.15);
        //park();
            }
            else if (left_color.blue() > 1.5) {
                //add code for specific position/encoder ticks forward
                left_beacon.setPosition(.15);
                //park();
            }
        }

        public void park(){
            //enter some code about how to park
        }

    @Override
    public void runOpMode() {
        telemetry.addData("Status first", "Initialized");
        telemetry.update();
        left_color = hardwareMap.colorSensor.get("left_color");
        right_color = hardwareMap.colorSensor.get("right_color");
        floor_seeker=hardwareMap.colorSensor.get("floor_seeker");
        left_color.setI2cAddress(leftColorI2c);
        right_color.setI2cAddress(rightColorI2c);
        floor_seeker.setI2cAddress(floorI2c);
        floor_seeker.enableLed(false);
        floor_seeker.enableLed(true);
        right_color.enableLed(false);
        left_color.enableLed(false);
        left_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_range");
        left_range.setI2cAddress(leftRangeI2c);
        left_drive1 = hardwareMap.dcMotor.get("left_drive1");
        left_drive2 = hardwareMap.dcMotor.get("left_drive2");
        right_drive1 = hardwareMap.dcMotor.get("right_drive1");
        right_drive2 = hardwareMap.dcMotor.get("right_drive2");
        right_drive2.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mortar = hardwareMap.dcMotor.get("mortar");
        mortar.setDirection(DcMotorSimple.Direction.REVERSE);
        mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mortar_gate = hardwareMap.servo.get("mortar_gate");
        cap_ball_lift = hardwareMap.dcMotor.get("cap_ball_lift");
        cap_ball_tilt = hardwareMap.dcMotor.get("cap_ball_tilt");
        cap_ball_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap_ball_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cap_ball_tilt.setPower(0.3);
        left_beacon = hardwareMap.servo.get("left_beacon");
        right_beacon = hardwareMap.servo.get("right_beacon");
        right_beacon.setDirection(Servo.Direction.REVERSE);
        left_beacon.setPosition(0.19);
        right_beacon.setPosition(0.26);
        particle_collector = hardwareMap.dcMotor.get("particle_collector");
        collector_gate = hardwareMap.servo.get("collector_gate");
        magazine_cam = hardwareMap.servo.get("magazine_cam");
        gyro = hardwareMap.gyroSensor.get("gyro");
        collector_gate.setPosition(PCGateDown);
        mortar_gate.setPosition(mortarGateDown);
        magazine_cam.setPosition(camZero);
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            sleep(40);
        }

        waitForStart();
        resetStartTime();
        positionToShoot();
        sleep(1000);
        //shootBall();
        turnToWall();
        driveStraight();
        turnNormal();
        findWhiteLine();
        //sleep(1000);
        //firstBeaconPress();
        //capBall();
        //shoot(30);
        wallSense();
    }
}