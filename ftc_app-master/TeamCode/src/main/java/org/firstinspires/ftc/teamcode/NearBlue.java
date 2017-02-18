
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
@Autonomous(name="NearBlueNoWait", group="Auto")
public class NearBlue extends LinearOpMode {
    mordorHardware robot           = new mordorHardware();
    double turnTolerance;
    double currentHeading;
    double powerFloor;
    double floor = .06;
    double driveGain = .0000;
    int distance = 1450;
    double headingError;
    double driveSteering;
    double leftPower;
    double rightPower;
    double firingSpeed = .9;
    int mortarFreeState = 1440;
    int driveDistance = (470);
    boolean correctColor1 = false;
    boolean correctColor2 = false;
    boolean firstPress =true;




    public void resetEncoders(){
        robot.right_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void positionToShoot() {
        resetEncoders();
        robot.magazine_cam.setPosition(robot.camUp);
        while (robot.left_drive1.getCurrentPosition()<driveDistance||robot.left_drive2.getCurrentPosition()<driveDistance||robot.right_drive1.getCurrentPosition()<driveDistance||robot.right_drive2.getCurrentPosition()<driveDistance&&opModeIsActive()) {

            robot.setPowerLeft(.2);
            robot.setPowerRight(.2);
            robot.waitForTick(10);
        }
        robot.stopMotors();
    }
    public void hitBeacon(){
        robot.setPowerLeft(.15);
        robot.setPowerRight(.15);
        sleep(1500);
        robot.stopMotors();
    }
    public void backUp(){
        robot.setPowerLeft(-.1);
        robot.setPowerRight(-.1);
        sleep(500);
        robot.stopMotors();

    }
    public void detectColor(){
        if (robot.frontColor.blue() > robot.frontColor.red()) {
            correctColor1 =true;
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


    public void findWhiteLine() {
        while (robot.floor_seeker.green() < 6&&opModeIsActive()) {
            robot.left_drive1.setPower(0.2);
            robot.left_drive2.setPower(0.2);
            robot.right_drive1.setPower(0.2);
            robot.right_drive2.setPower(0.2);
            //wallSense();
        }
        robot.stopMotors();

    }

    public void firstBeaconPress() {
        robot.turnRight(75, .03);
        sleep(500);
        while(!correctColor1&&opModeIsActive()) {
            if(!firstPress){
                sleep(4000);
            }
            hitBeacon();
            sleep(500);
            detectColor();
            backUp();
            firstPress = false;
        }
    }
    public void secondBeaconPress(){
        robot.turnRight(80, .03);
        sleep(500);
        firstPress =true;
        while(!correctColor2&&opModeIsActive()) {
            if(!firstPress){
                sleep(4000);
            }
            hitBeacon();
            sleep(500);
            detectColor();
            backUp();
            firstPress = false;
        }


    }
    public void driveStraight(){
        resetEncoders();
        while(robot.left_drive1.getCurrentPosition()<distance||robot.left_drive2.getCurrentPosition()<distance||robot.right_drive1.getCurrentPosition()<distance||robot.right_drive2.getCurrentPosition()<distance&&opModeIsActive()){
            robot.setPowerRight(.2);
            robot.setPowerLeft(.2);
            robot.waitForTick(10);
        }
        robot.stopMotors();
    }

    public void turnToWall(){
        turnTolerance = 2;
        robot.turnRight(47, .1);
        sleep(500);
    }
    public void turnNormal(){
        turnTolerance = 2;
        robot.turnLeft(10,.05);
        sleep(500);
    }
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        resetStartTime();
        positionToShoot();
        sleep(500);
        //shootBall();
        sleep(500);
        turnToWall();
        driveStraight();
        turnNormal();
        findWhiteLine();
        sleep(500);
        firstBeaconPress();
        robot.turnLeft(10, .05);
        driveStraight();
        findWhiteLine();
        secondBeaconPress();
        //capBall();
    }
}