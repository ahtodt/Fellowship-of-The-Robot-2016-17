package org.firstinspires.ftc.robotcontroller.external.samples;

import org.firstinspires.ftc.robotcontroller.external.samples.MRI_Range_Sensors;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

/**
 * Created by OneFourTw on 2/11/2017.
 */

public class I2CTest extends OpMode {
    DcMotor test1;
    I2cAddr left_rangeI2c = I2cAddr.create8bit(0x30);
    I2cDevice left_range;

    public void init(){
        test1 = hardwareMap.dcMotor.get("test1");
        test1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_range = hardwareMap.get(I2cDevice.class, "left_range");
        //left_range.setI2cAddress(left_rangeI2c);
    }

    public void loop(){
      //  telemetry.addData("Range Optical", left_range.g());
        // telemetry.addData("Range Ultrasonic", left_range.cmUltrasonic());

        test1.setPower(1);
        test1.setTargetPosition(150);


    }

}
