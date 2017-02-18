
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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


//delay, longer drive before shoot, turn for square after shoot, only shoot one or fix the mortar
@Autonomous(name="AutoNear", group="Auto")
public class AutoNear extends LinearOpMode {
    mordorHardware robot           = new mordorHardware();
    double turnTolerance;
    double currentHeading;
    double powerFloor;
    double floor = .015;
    double driveGain = .0003;
    int distance = 1450;
    double headingError;
    double driveSteering;
    double leftPower;
    double rightPower;
    double firingSpeed = .9;
    int mortarFreeState = 1440;
    int driveDistance = (525);
    double target = 20;


    public void positionToShoot() {
        robot.right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.magazine_cam.setPosition(robot.camUp);
        while (robot.right_drive1.getCurrentPosition() < driveDistance && robot.left_drive2.getCurrentPosition() < driveDistance&&opModeIsActive()) {

            robot.setPowerLeft(.2);
            robot.setPowerRight(.2);
        }
        robot.stopMotors();
    }


    public void waitForTen() {
        while ((getRuntime() < 5) && opModeIsActive()) {
        }
    }

    public void shootBall() {
        robot.mortar.setPower(firingSpeed);
        robot.mortar.setTargetPosition(mortarFreeState);
        robot.mortar_gate.setPosition(robot.mortarGateDown);
        sleep(500);
        robot.mortar_gate.setPosition(robot.mortarGateUp);
        sleep(1000);
        robot.mortar_gate.setPosition(robot.mortarGateDown);
        robot.mortar.setPower(firingSpeed);
        robot.mortar.setTargetPosition(mortarFreeState * 2);
    }

    public void capBall() {
        while (robot.right_drive1.getCurrentPosition() < (driveDistance * 2.25)
                && robot.left_drive2.getCurrentPosition() < (driveDistance * 2.25)&&opModeIsActive()) {
            robot.left_drive1.setPower(.1);
            robot.left_drive2.setPower(.1);
            robot.right_drive1.setPower(.2);
            robot.right_drive2.setPower(.2);
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
        robot.stopMotors();

    }


    public void findWhiteLine() {
        robot.right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (robot.floor_seeker.green() < 6&&opModeIsActive()) {
            robot.left_drive1.setPower(0.1);
            robot.left_drive2.setPower(0.1);
            robot.right_drive1.setPower(0.1);
            robot.right_drive2.setPower(0.1);
            //wallSense();
        }
        robot.stopMotors();

    }
    public void gyroTurnLeft(double desiredAngle) {
        do {

            if (robot.gyro.getHeading() > 180) {
                currentHeading = robot.gyro.getHeading() - 360;
            }else{
                currentHeading = robot.gyro.getHeading();
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

            robot.setPowerLeft(leftPower);
            robot.setPowerRight(rightPower);

    } while (Math.abs(currentHeading-desiredAngle)>turnTolerance&&opModeIsActive());

        robot.stopMotors();
}
    public void wallTurn(int target){
        robot.stopMotors();
        robot.right_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setPowerLeft(.1);
        robot.setPowerRight(.1);
        robot.left_drive1.setTargetPosition(-target);
        robot.left_drive2.setTargetPosition(-target);
        robot.right_drive1.setTargetPosition(target);
        robot.right_drive2.setTargetPosition(target);
            while (robot.left_drive1.getCurrentPosition() > -target && opModeIsActive()) {


        }
    }
    public void lineTurn(int angle){
        robot.stopMotors();
        robot.right_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_drive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left_drive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setPowerLeft(.1);
        robot.setPowerRight(.1);
        robot.left_drive1.setTargetPosition(-angle+20);
        robot.left_drive2.setTargetPosition(-angle+20);
        robot.right_drive1.setTargetPosition(angle+10);
        robot.right_drive2.setTargetPosition(angle+10);
        while (robot.left_drive1.getCurrentPosition() < (-angle+20) && opModeIsActive()) {


        }
    }
        public void firstBeaconPress() {
            robot.stopMotors();
            if (robot.frontColor.red() > 1.5) {
                robot.left_beacon.setPosition(robot.leftBeaconOut);
               smallMove1();
                sleep(500);
                robot.left_beacon.setPosition(robot.leftBeaconIn);
                smallMove2();
            } else if (robot.frontColor.blue() > 1.5) {
                //add code for specific position/encoder ticks forward
                smallMove1();
                sleep(500);
                robot.left_beacon.setPosition(robot.leftBeaconOut);
                smallMove2();
                sleep(500);
                robot.left_beacon.setPosition(robot.leftBeaconIn);
            }else{
                forward();
            }
            robot.stopMotors();
        }
    public void smallMove1(){
        int encoderStart1 = robot.right_drive1.getCurrentPosition();
        while(robot.right_drive1.getCurrentPosition()<encoderStart1+70&&opModeIsActive()){
            robot.setPowerLeft(.05);
            robot.setPowerRight(.05);
        }

    }
    public void smallMove2(){

        int encoderStart2 = robot.right_drive1.getCurrentPosition();
        while(robot.right_drive1.getCurrentPosition()<encoderStart2+70&&opModeIsActive()){
            robot.setPowerLeft(.05);
            robot.setPowerRight(.05);
        }

    }

    public void forward(){

        int encoderStart = robot.right_drive1.getCurrentPosition();
        while(robot.right_drive1.getCurrentPosition()<encoderStart+100&&opModeIsActive()){
            robot.setPowerLeft(.1);
            robot.setPowerRight(.1);
        }
    }
    public void turnToWhiteLine(double desiredAngle) {
        do {

            if (robot.gyro.getHeading() > 180) {
                currentHeading = robot.gyro.getHeading() - 360;
            }else{
                currentHeading = robot.gyro.getHeading();
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

            robot.setPowerLeft(leftPower);
            robot.setPowerRight(rightPower);

        } while (Math.abs(currentHeading-desiredAngle)>turnTolerance&&opModeIsActive());

        robot.stopMotors();
    }
    public void driveStraight(){
        robot.right_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(robot.left_drive1.getCurrentPosition()<distance||robot.left_drive2.getCurrentPosition()<distance||robot.right_drive1.getCurrentPosition()<distance||robot.right_drive2.getCurrentPosition()<distance&&opModeIsActive()){
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
            robot.setPowerRight(.2);
            robot.setPowerLeft(.2);
        }
        robot.stopMotors();
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
        robot.right_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

        public void park(){
            //enter some code about how to park
        }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

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
    }
}