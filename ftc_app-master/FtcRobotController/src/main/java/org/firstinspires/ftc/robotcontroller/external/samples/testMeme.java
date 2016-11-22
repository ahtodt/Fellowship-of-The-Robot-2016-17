
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

public class testMeme extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;
    double left;
    double right;
    double turnTolerance = 5;
    double newHeading;
    double oneRotation =420;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor shooterMotor;
    GyroSensor gyro;
    ModernRoboticsI2cRangeSensor Range;
    ModernRoboticsI2cRangeSensor rightRangeSensor;
    ColorSensor floorSeeker;


    public void driveStraight(){
        while(Range.cmUltrasonic()>30){
            if(gyro.getHeading()>180&&gyro.getHeading()<=360){
                leftMotor.setPower(.8);
                rightMotor.setPower(.2);
            }else if(gyro.getHeading()>0&&gyro.getHeading()<=180){
                rightMotor.setPower(.8);
                leftMotor.setPower(.2);
            }else if(gyro.getHeading()==0){
                rightMotor.setPower(.8);
                leftMotor.setPower(.8);
            }
            sleep(10);
        }
    }

    public void wallSense(){
        while(floorSeeker.green() < 10){
            if(rightRangeSensor.cmUltrasonic()<15){
                rightMotor.setPower(.8);
                leftMotor.setPower(.3);
            }else if(rightRangeSensor.cmUltrasonic()>15){
                leftMotor.setPower(.8);
                rightMotor.setPower(.3);
            }
            sleep(40);
        }

    }


    public void shoot(int shots){
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



    }


    /*public void gyroTurn(int desiredAngle,double power, String direction) {

         if (direction == "left") {

             while(Math.abs(desiredAngle-newHeading)< turnTolerance){
                 leftMotor.setPower(midSpeed+adjust);
                 rightMotor.setPower(midSpeed-adjust);
             }




         }
         if (direction == "right") {
             while (Math.abs(desiredAngle -gyro.getHeading())>5) {

                 leftMotor.setPower(power);
                 rightMotor.setPower(-power);
                 telemetry.addData("gyro", gyro.getHeading());
             }
         }
     }*/
    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftMotor=hardwareMap.dcMotor.get("left_drive");
        rightMotor=hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        gyro=hardwareMap.gyroSensor.get("gyro");
        Range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        floorSeeker = hardwareMap.colorSensor.get("floorSeeker");
        rightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRangeSensor");

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
        //gyroTurn(270, .6, "left");
        // wallSense();
        shoot(30);

        rightMotor.setPower(0);
        leftMotor.setPower(0);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            sleep(40);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            telemetry.addData("range", Range.cmUltrasonic());
            telemetry.addData("gyro", gyro.getHeading());
            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
        }
    }
}