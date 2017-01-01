
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
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Math.*;


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

public class FirstAuto extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;
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
    DcMotor left_drive1;
    DcMotor right_drive1;
    DcMotor left_drive2;
    DcMotor right_drive2;
    //DcMotor shooterMotor;
    GyroSensor gyro;
    //ModernRoboticsI2cRangeSensor front_range;
    ModernRoboticsI2cRangeSensor right_range;
    ModernRoboticsI2cRangeSensor left_range;
    ColorSensor floor_seeker;
    ColorSensor left_beacon;
    //ColorSensor right_beacon;

    //560 encoder ticks for AndyMark motors, 1440 for Tetrix

    public void stopMotors(){
        right_drive1.setPower(0);
        left_drive1.setPower(0);
        right_drive2.setPower(0);
        left_drive2.setPower(0);
    }
    public void driveStraight() {

            currentHeading = gyro.getHeading();

            if (gyro.getHeading() > 180) {
                currentHeading = currentHeading - 360;
            }

            if (currentHeading > startHeading) {
                left_drive1.setMaxSpeed(56);
                left_drive2.setMaxSpeed(56);
                right_drive1.setMaxSpeed(112);
                right_drive2.setMaxSpeed(112);
            } else if (currentHeading<startHeading ){
                left_drive1.setMaxSpeed(112);
                left_drive2.setMaxSpeed(112);
                right_drive1.setMaxSpeed(56);
                right_drive2.setMaxSpeed(56);
            } else if (currentHeading == startHeading) {
                left_drive1.setMaxSpeed(112);
                left_drive2.setMaxSpeed(112);
                right_drive1.setMaxSpeed(112);
                right_drive2.setMaxSpeed(112);
            }
    }

    public void wallSense(){
        while(floor_seeker.green() < 5){
            int target = 38;
            double distance = right_range.cmUltrasonic();
            double error = distance - target;
            double differential = error * 0.1;
            double velocity = right_drive1.getMaxSpeed();
            double rightWheelVelocity = velocity + differential;
            double leftWheelVelocity = velocity - differential;
            //figure out how to round right/leftWheelVelocity to produce an int


            if(distance < target){
                //must have an int for velocity, can have either for power (can't have 2.54 encoder ticks)
                right_drive1.setPower(rightWheelVelocity);
                right_drive2.setMaxSpeed(rightWheelVelocity);
                left_drive1.setMaxSpeed(leftWheelVelocity);
                left_drive2.setMaxSpeed(leftWheelVelocity);
            }
            else if(distance > target){
                left_drive1.setMaxSpeed(leftWheelVelocity);
                left_drive2.setMaxSpeed(leftWheelVelocity);
                right_drive1.setMaxSpeed(rightWheelVelocity);
                right_drive2.setMaxSpeed(rightWheelVelocity);
            }
            else if(distance == target){
                left_drive1.setMaxSpeed(leftWheelVelocity);
                left_drive2.setMaxSpeed(leftWheelVelocity);
                right_drive1.setMaxSpeed(rightWheelVelocity);
                right_drive2.setMaxSpeed(rightWheelVelocity);
            }
            telemetry.addData("rightRange", right_range.cmUltrasonic());
            telemetry.update();
        }
    }


    /*public void gyroTurn(double desiredAngle) {

        do {
            currentHeading = gyro.getHeading();
            if (gyro.getHeading() > 180) {
                currentHeading = currentHeading - 360;
            }
            headingError = desiredAngle - currentHeading;
            driveSteering = headingError * driveGain;

            leftPower = driveSteering;
            if(leftPower>.9){
                leftPower = .9;
            }else if(leftPower<-.9){
                leftPower = -.9;
            }

            rightPower = -driveSteering;
            if(rightPower>.9){
                rightPower = .9;
            }else if(rightPower<-.9){
                rightPower = -.9;
            }

            left_drive1.setPower(leftPower);
            left_drive2.setPower(leftPower);
            right_drive1.setPower(rightPower);
            right_drive2.setPower(rightPower);

            telemetry.addData("in turn loop", leftPower);

        } while (Math.abs(currentHeading-desiredAngle)>turnTolerance);

        stopMotors();
    }*/

    public void findWhiteLine(){
        currentHeading = gyro.getHeading();
        if (gyro.getHeading() > 180) {
            currentHeading = currentHeading - 360;
        }
        startHeading = currentHeading;
            while(floor_seeker.green()<5){
                driveStraight();
            }

        stopMotors();

    }

    /*public void driveToWall(){
        currentHeading = gyro.getHeading();
        if (gyro.getHeading() > 180) {
            currentHeading = currentHeading - 360;
        }
        startHeading = currentHeading;
        while(front_range.cmUltrasonic()>38){
            driveStraight();
        }
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

    @Override
    public void runOpMode(){
        telemetry.addData("Status first", "Initialized");
        telemetry.update();
        left_drive1=hardwareMap.dcMotor.get("left_drive1");
        left_drive2=hardwareMap.dcMotor.get("left_drive2");
        right_drive1=hardwareMap.dcMotor.get("right_drive1");
        right_drive2=hardwareMap.dcMotor.get("right_drive2");
        right_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        //shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        gyro=hardwareMap.gyroSensor.get("gyro");
        //front_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front_range");
        floor_seeker = hardwareMap.colorSensor.get("floor_seeker");
        right_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "right_range");

        // Wait for the game to start (driver presses PLAY)
        gyro.calibrate();
        while(gyro.isCalibrating()){
            sleep(40);
        }




        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
                // leftMotor  = hardwareMap.dcMotor.get("left_drive");
                // rightMotor = hardwareMap.dcMotor.get("right_drive");

                // eg: Set the drive motor directions:
                // "Reverse" the motor that runs backwards when connected directly to the battery
                // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
                // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

                // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        //driveStraight();
        //driveToWall();
             //gyroTurn(-90);
        findWhiteLine();
        wallSense();
        //shoot(30);
        right_drive1.setPower(0);
        right_drive2.setPower(0);
        left_drive1.setPower(0);
        left_drive2.setPower(0);



        //run until the end of the match (driver presses STOP)
        //while (opModeIsActive()) {
        for (int i=1;i<100000;i++) {
            // telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.update();
            //telemetry.addData("range blah", front_range.cmUltrasonic());
            telemetry.update();
            sleep(20);
            // telemetry.addData("gyro", gyro.getHeading());
            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
        }
    }
}

