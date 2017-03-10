
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
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


//delay, longer drive before shoot, turn for square after shoot, only shoot one or fix the mortar
@Autonomous(name="BlueBeacons", group="Auto")
public class BlueBeacons extends LinearOpMode {
    mordorHardware robot           = new mordorHardware();
    double turnTolerance;
    double currentHeading;
    double drivingHeading;
    double powerFloor;
    double floor = .06;
    double driveGain = .0000;
    int distance = 1200;
    double headingError;
    double driveSteering;
    double leftPower;
    double rightPower;
    double firingSpeed = .9;
    double difference;
    double angleDelta;
    double newPower;
    double adjustment;
    double correctionFactor = .005;
    int mortarFreeState = 1440;
    int driveDistance = (433);
    boolean correctColor1 = false;
    boolean correctColor2 = false;
    boolean firstPress =true;



    public void turnLeft(int angle, double power){
        do {

            if (robot.getAdafruitHeading() > 180) {
                currentHeading = robot.getAdafruitHeading() - 360;
            } else {
                currentHeading = robot.getAdafruitHeading();
            }
            angleDelta = currentHeading-angle;
            adjustment = angleDelta*correctionFactor;
            if(adjustment>.1){
                adjustment=.1;
            }
            newPower = power+adjustment;

            robot.setPowerRight(newPower);
            robot.setPowerLeft(-newPower);
            robot.waitForTick(10);
        } while(currentHeading>angle&&opModeIsActive());
        robot.stopMotors();
    }
    public void turnRight(int angle, double power){
        do {

            if (robot.getAdafruitHeading() > 180) {
                currentHeading = robot.getAdafruitHeading() - 360;
            } else {
                currentHeading = robot.getAdafruitHeading();
            }
            robot.setPowerRight(-power);
            robot.setPowerLeft(power);
            robot.waitForTick(10);
        } while(currentHeading<angle&&opModeIsActive());
        robot.stopMotors();
    }

    public void resetMaxSpeed(){

        robot.left_drive1.setMaxSpeed(9000);
        robot.left_drive2.setMaxSpeed(9000);
        robot.right_drive1.setMaxSpeed(9000);
        robot.right_drive2.setMaxSpeed(9000);
    }
    public void positionToShoot() {
        robot.resetEncoders();
        robot.magazine_cam.setPosition(robot.camUp);
        while (robot.left_drive1.getCurrentPosition()<driveDistance||robot.left_drive2.getCurrentPosition()<driveDistance||robot.right_drive1.getCurrentPosition()<driveDistance||robot.right_drive2.getCurrentPosition()<driveDistance&&opModeIsActive()) {

            robot.setPowerLeft(.2);
            robot.setPowerRight(.2);
            robot.waitForTick(10);
        }
        robot.stopMotors();
    }
    public void hitBeacon(){
        robot.setPowerLeft(.22);
        robot.setPowerRight(.22);
        sleep(1050);
        robot.stopMotors();
    }
    public void smallBack(){
        robot.setPowerLeft(-.2);
        robot.setPowerRight(-.2);
        sleep(100);
        robot.stopMotors();
    }
    public void backUp(){
        robot.setPowerLeft(-.2);
        robot.setPowerRight(-.2);
        sleep(400);
        robot.stopMotors();

    }
    public void detectColor1(){
        if (robot.frontColor.blue() > robot.frontColor.red()) {
            correctColor1 =true;
        }
    }
    public void detectColor2(){
        if (robot.frontColor.blue() > robot.frontColor.red()) {
            correctColor2 =true;
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
        }
        robot.stopMotors();

    }

    public void predrive(){
        robot.setPowerLeft(.4);
        robot.setPowerRight(.4);
        robot.left_drive1.setMaxSpeed(1600);
        robot.left_drive2.setMaxSpeed(1600);
        robot.right_drive1.setMaxSpeed(1600);
        robot.right_drive2.setMaxSpeed(1600);
        sleep(2000);
        robot.stopMotors();
    }

    public void findWhiteLine1(int angle, double basePower) {
        //speed this up
        double rightBasePower = 1.1*basePower;
        while (robot.floor_seeker.green() < 1200&&opModeIsActive()) {
            if (robot.getAdafruitHeading() > 180) {
                drivingHeading = robot.getAdafruitHeading() - 360;
            } else {
                drivingHeading = robot.getAdafruitHeading();
            }
            difference = angle - drivingHeading;
            if(difference>5){
                difference =5;
            }else if (difference<-5){
                difference = -5;
            }
            if(difference>0){
                //less power from right
                robot.setPowerLeft(basePower);
                robot.setPowerRight(rightBasePower-(difference*rightBasePower*correctionFactor));
            }else if(difference<0){
                //less power from left
                robot.setPowerLeft(basePower+(difference*basePower*correctionFactor));
                robot.setPowerRight(rightBasePower);
            }else{
                robot.setPowerLeft(basePower);
                robot.setPowerRight(rightBasePower);
            }
        }
        robot.stopMotors();
    }
    public void findWhiteLine2() {
        //speed this up
        while (robot.floor_seeker.green() < 6&&opModeIsActive()) {
            robot.setPowerLeft(.1);
            robot.setPowerRight(.1);
        }
        robot.stopMotors();
    }
    public void firstBeaconPress() {
        turnRight(40, .13);
        sleep(500);
        while(!correctColor1&&opModeIsActive()) {
            if(!firstPress){
                sleep(3500);
            }
            hitBeacon();
            smallBack();
            sleep(1500);
            detectColor1();
            backUp();
            firstPress = false;
        }
    }
    public void secondBeaconPress(){
        turnRight(40, .13);
        sleep(500);
        firstPress =true;
        while(!correctColor2&&opModeIsActive()) {
            if(!firstPress){
                sleep(3200);
            }
            hitBeacon();
            smallBack();
            sleep(1500);
            detectColor2();
            backUp();
            firstPress = false;
        }


    }
    public void driveStraight(){
        robot.resetEncoders();
        while(robot.left_drive1.getCurrentPosition()<distance||robot.left_drive2.getCurrentPosition()<distance||robot.right_drive1.getCurrentPosition()<distance||robot.right_drive2.getCurrentPosition()<distance&&opModeIsActive()){
            robot.setPowerRight(.25);
            robot.setPowerLeft(.25);

            robot.waitForTick(10);
        }
        robot.stopMotors();
    }

    public void turnToWall(){
        turnRight(46, .13);
        sleep(500);
    }
    public void turnNormal(){
        turnLeft(10,.1);
        sleep(500);
    }
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        //predrive();
        //resetMaxSpeed();
        findWhiteLine1(0, .13);
        resetMaxSpeed();
        sleep(100);
        firstBeaconPress();
        backUp();
        turnLeft(5, .1);
        predrive();
        resetMaxSpeed();
        findWhiteLine1(-30, .13);
        resetMaxSpeed();
        secondBeaconPress();
        //capBall();
    }
}