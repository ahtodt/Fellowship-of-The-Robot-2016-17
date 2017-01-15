

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


public class RedorBlu extends LinearOpMode {
    String color;



    @Override
    public void runOpMode() {

        while(!gamepad1.a || !gamepad1.x){
            telemetry.addLine("red or blue?");`
        }
        if (gamepad1.a){
            color = "blue";
        }
        if(gamepad1.x){
            color = "red";
        }
        telemetry.addLine("color selected =");
        telemetry.addLine(color);
        waitForStart();

    }




}


