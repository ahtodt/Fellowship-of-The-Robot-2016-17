
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
        telemetry.addData("blue", robot.floor_seeker.blue());
        telemetry.addData("red", robot.floor_seeker.red());
        telemetry.addData("green", robot.floor_seeker.green());
    }
}


