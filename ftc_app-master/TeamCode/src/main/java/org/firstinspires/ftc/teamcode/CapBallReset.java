

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="CapBallReset", group="TeleOp")
public class CapBallReset extends OpMode {
    mordorHardware robot = new mordorHardware();
    @Override
    public void init() {
    robot.init(hardwareMap);
    }
    @Override
    public void loop() {
        if(gamepad1.dpad_down){
            robot.cap_ball_lift.setPower(1);
        } else if(gamepad1.dpad_up){
            robot.cap_ball_lift.setPower(-1);
        } else{
            robot.cap_ball_lift.setPower(0);
        }
    }
}


