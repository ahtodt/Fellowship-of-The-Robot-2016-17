
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ColorSensorTest", group = "TeleOp")
public class colorSensorTest extends OpMode {
    mordorHardware robot = new mordorHardware();



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("blue", robot.frontColor.blue());
        telemetry.addData("red", robot.frontColor.red());
    }
}


