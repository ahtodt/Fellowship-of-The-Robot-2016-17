
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
@Autonomous(name="AutoFarWait", group="Auto")
public class AutoFarWait extends LinearOpMode {
    mordorHardware robot           = new mordorHardware();
    private ElapsedTime runtime = new ElapsedTime();
    double left;
    double right;
    double turnTolerance = 1;
    double newHeading;
    double oneRotation = 420;
    double currentHeading;
    double driveGain = .007;
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
    int mortarFreeState = 1440;
    int mortarEngagedState = 300;
    int driveDistance = (525);
    int shots = 1;


    public void stopMotors() {
        robot.right_drive1.setPower(0);
        robot.left_drive1.setPower(0);
        robot.right_drive2.setPower(0);
        robot.left_drive2.setPower(0);
    }

    public void setPowerLeft(double power) {
        robot.left_drive1.setPower(power);
        robot.left_drive2.setPower(power);
    }

    public void setPowerRight(double power) {
        robot.right_drive1.setPower(power);
        robot.right_drive2.setPower(power);
    }

    public void positionToShoot() {
        robot.right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.magazine_cam.setPosition(camUp);
        while (robot.right_drive1.getCurrentPosition() < driveDistance && robot.left_drive2.getCurrentPosition() < driveDistance&&opModeIsActive()) {

            setPowerLeft(.1);
            setPowerRight(.1);
        }
        stopMotors();
    }


    public void waitForTen() {
        while ((getRuntime() < 18) && opModeIsActive()) {
            sleep(500);
        }
    }

    public void shootBall() {
        robot.mortar.setPower(firingSpeed);
        robot.mortar.setTargetPosition(mortarFreeState);
        robot.mortar_gate.setPosition(mortarGateDown);
        sleep(500);
        robot.mortar_gate.setPosition(mortarGateUp);
        sleep(1000);
        robot.mortar_gate.setPosition(mortarGateDown);
        robot.mortar.setPower(firingSpeed);
        robot.mortar.setTargetPosition(mortarFreeState * 2);
    }

    public void capBall() {
        while (robot.right_drive1.getCurrentPosition() < (driveDistance * 3)
                //note: if you think you need to set it to *2, make it *3 and if you want *3, make it *4, etc.
                && robot.left_drive2.getCurrentPosition() < (driveDistance * 3)&&opModeIsActive()) {
            robot.left_drive1.setPower(.15);
            robot.left_drive2.setPower(.15);
            robot.right_drive1.setPower(.15);
            robot.right_drive2.setPower(.15);
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
        while (robot.floor_seeker.blue() < 12) {
            robot.left_drive1.setPower(0.1);
            robot.left_drive2.setPower(0.1);
            robot.right_drive1.setPower(0.1);
            robot.right_drive2.setPower(0.1);
            telemetry.addData("sensor", robot.floor_seeker.blue());
            //wallSense();
        }
        stopMotors();
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
        }

    //stopMotors();

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

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        waitForStart();
        resetStartTime();
        sleep(10000);
        positionToShoot();
        shootBall();
        sleep(10000);
        capBall();

        //shoot(30);

    }
}