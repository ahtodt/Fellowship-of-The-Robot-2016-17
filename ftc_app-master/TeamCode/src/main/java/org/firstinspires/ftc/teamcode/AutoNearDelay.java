
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
@Autonomous(name="AutoNearWait", group="Auto")
public class AutoNearDelay extends LinearOpMode {
    mordorHardware robot           = new mordorHardware();
    double firingSpeed = .9;
    int mortarFreeState = 1440;
    int driveDistance = (525);


    public void positionToShoot() {
        robot.right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.magazine_cam.setPosition(robot.camUp);
        while (robot.right_drive1.getCurrentPosition() < driveDistance && robot.left_drive2.getCurrentPosition() < driveDistance&&opModeIsActive()) {

            robot.setPowerLeft(.1);
            robot.setPowerRight(.1);
        }
        robot.stopMotors();
    }


    public void waitForTen() {
        while ((getRuntime() < 18) && opModeIsActive()) {
            sleep(500);
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
        while (robot.right_drive1.getCurrentPosition() < (driveDistance * 2.5)
                && robot.left_drive2.getCurrentPosition() < (driveDistance * 2.5)&&opModeIsActive()) {
            robot.left_drive1.setPower(.1);
            robot.left_drive2.setPower(.1);
            robot.right_drive1.setPower(.1);
            robot.right_drive2.setPower(.1);
        }
        robot.stopMotors();

    }


    public void findWhiteLine() {
        while (robot.floor_seeker.blue() < 12) {
            robot.left_drive1.setPower(0.1);
            robot.left_drive2.setPower(0.1);
            robot.right_drive1.setPower(0.1);
            robot.right_drive2.setPower(0.1);
        }
        robot.stopMotors();

    }



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