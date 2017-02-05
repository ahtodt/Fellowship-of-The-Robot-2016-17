

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;


public class RedorBlu extends LinearOpMode {
    String color;



    @Override
    public void runOpMode() {
        while(opModeIsActive()){
            telemetry.addLine("red or blue?");
            if(gamepad1.x){color = "red"};
        if (gamepad1.a){color = "blue"};
            telemetry.addLine(color);
}









