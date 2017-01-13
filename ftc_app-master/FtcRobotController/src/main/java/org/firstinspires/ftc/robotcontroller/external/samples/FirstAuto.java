
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
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
    double target = 20;
    double distance = right_range.cmUltrasonic();
    double error = distance - target;
    double differential = error * 0.1;
    double rightVelocity = right_drive1.getPower(); //reversed here but still resulted in same turn, so obviously
    double leftVelocity = left_drive1.getPower(); //the error occurs before the power goes toward the motors
    double leftVelocityCorrection = leftVelocity + differential; //telemetry for range sensors? telemetry for error?
    double rightVelocityCorrection = rightVelocity + differential; //telemetry for differential?
    /*int rightVelocityCorrectionInt = (int)Math.round(rightVelocityCorrection);
    int leftVelocityCorrectionInt = (int)Math.round(leftVelocityCorrection);*/

    //560 encoder ticks for AndyMark motors, 1440 for Tetrix, 1680 for AndyMark 60:1
    //WRITE SOME CODE SO THAT THE MOTORS DON'T BURN OUT IF THE ROBOT RUNS INTO SOMETHING IMMOBILE
    //changed + to - and - to + above, changed 38 to 25, added telemetry into if statement
    //reason code gave errors because wallSense written before hardwareMap
    //changed + and - but still turns same way (turns clockwise from the top, left drive forward)
    //just changed so + and + for both

    //possible problems: range sensor not at pivot point, gets further away no matter what
    //wrong target position or need to use optical not ultrasonic
    //not gaining enough speed prior to reading range/adjusting (put delay?)
    //I think I will try to get a video of how its acting so I know which side is being powered in which direction. I will also try to
    //measure how far away the range sensor is from the wall.

    //something to do with velocity control
    //just commented out velocity control. I don't think this is the issue though, so I'm going to change something else before I test
    //really hoping that the + + thing will fix the problem since its possible each side drives in opposite directions
    //++ didn't work. will try to read left_range instead of right_range
    //never did this. tried making sure using "real" configuration and made sure it matches wiring
    //threw "NullPointerException double com.qualcomm.hardware.modernroboticsi2crangesensor.cmultrasonic" error
    //commented out the telemetry in the if statement, added cap_ball_tilt into code since it was missing
    //didn't get rid of error
    //added telemetry for target, rightRange, distance, error, differential, rightVelocity, leftVelocity, rightVelocityCorrection and
    //leftVelocityCorrection
    //doesn't have error with Reed's version of this code


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
    }*/
    /*public void shootBall(){
        mortar.setPower(firingSpeed);
        mortar.setTargetPosition(mortarFreeState);
        sleep(1000);
        mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mortar.setPower(cockingSpeed);
        mortar.setTargetPosition(mortarEngagedState);

    }*/
    /*public void driveStraight() {

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
*/

    public void findWhiteLine(){
        if(floor_seeker.green()<5){
            wallSense();
           }
        else if (floor_seeker.green()>5){
            left_drive1.setPower(0);
            left_drive2.setPower(0);
            right_drive1.setPower(0);
            right_drive2.setPower(0);
            //redBeaconPress();
        }

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
        if(floor_seeker.green()<5){
            wallSense();
        }
        else if (floor_seeker.green()>5){
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

    /*public void findWhiteLine(){
        currentHeading = gyro.getHeading();
        if (gyro.getHeading() > 180) {
            currentHeading = currentHeading - 360;
        }
        startHeading = currentHeading;
            while(floor_seeker.green()<5){
                driveStraight();
            }

        stopMotors();

    }*/

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
    public void runOpMode() {
        telemetry.addData("Status first", "Initialized");
        telemetry.update();
        left_drive1 = hardwareMap.dcMotor.get("left_drive1");
        left_drive2 = hardwareMap.dcMotor.get("left_drive2");
        right_drive1 = hardwareMap.dcMotor.get("right_drive1");
        right_drive2 = hardwareMap.dcMotor.get("right_drive2");
        right_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        mortar = hardwareMap.dcMotor.get("mortar");
        left_beacon = hardwareMap.servo.get("left_beacon");
        right_beacon = hardwareMap.servo.get("right_beacon");
        right_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "right_range");
        left_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_range");
        front_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front_range");
        gyro = hardwareMap.gyroSensor.get("gyro");
        floor_seeker = hardwareMap.colorSensor.get("floor_seeker");
        mortar.setDirection(DcMotorSimple.Direction.REVERSE);
        mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_color = hardwareMap.colorSensor.get("left_color");
        right_color = hardwareMap.colorSensor.get("right_color");
        collector_gate = hardwareMap.servo.get("collector_gate");
        magazine_cam = hardwareMap.servo.get("magazine_cam");
        mortar_gate = hardwareMap.servo.get("mortar_gate");
        cap_ball_lift = hardwareMap.dcMotor.get("cap_ball_lift");
        particle_collector = hardwareMap.dcMotor.get("particle_collector");
        cap_ball_tilt = hardwareMap.dcMotor.get("cap_ball_tilt");

        // Wait for the game to start (driver presses PLAY)
        /*gyro.calibrate();
        while(gyro.isCalibrating()){
            sleep(40);
        }*/




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

        //positionToShoot();
        //shootBall();
        //driveStraight();
        //driveToWall();
        //gyroTurn(-90);
        //findWhiteLine();
        wallSense();
        //shoot(30);
        right_drive1.setPower(0);
        right_drive2.setPower(0);
        left_drive1.setPower(0);
        left_drive2.setPower(0);

    }

        //run until the end of the match (driver presses STOP)
        //while (opModeIsActive()) {
        public void wallSense(){
            if(right_range.cmUltrasonic()<20){
                right_drive1.setPower(.25);
                right_drive2.setPower(.25);
                left_drive1.setPower(.25);
                left_drive2.setPower(.25);
            }else if(right_range.cmUltrasonic()>=20){
                left_drive1.setPower(.25);
                left_drive2.setPower(.25);
                right_drive1.setPower(.25);
                right_drive2.setPower(.25);
           /* right_drive1.setPower(.2);
            right_drive2.setPower(.2);
            left_drive1.setPower(.2);
            left_drive1.setPower(.2);*/
            /*right_drive1.setMaxSpeed(1680);
            right_drive2.setMaxSpeed(1680);
            left_drive1.setMaxSpeed(1680);
            left_drive2.setMaxSpeed(1680);*/
           /* if(distance != target){
                telemetry.addData("target", target);
                telemetry.addData("rightRange", right_range.cmUltrasonic());
                telemetry.addData("distance", distance);
                telemetry.addData("error", error);
                telemetry.addData("differential", differential);
                telemetry.addData("rightVelocity", rightVelocity);
                telemetry.addData("leftVelocity", leftVelocity);
                telemetry.addData("leftVelocityCorrection", leftVelocityCorrection);
                telemetry.addData("rightVelocityCorrection", rightVelocityCorrection);
                telemetry.update(); */

                /*right_drive1.setPower(rightVelocityCorrection); //potentially reverse different motors in hardware map
                right_drive2.setPower(rightVelocityCorrection);
                left_drive1.setPower(leftVelocityCorrection);
                left_drive2.setPower(leftVelocityCorrection);*/
            }
        }

        //right_drive1.setPower(0);
        //right_drive2.setPower(0);
        //left_drive1.setPower(0);
        //left_drive2.setPower(0);

    }


//110