

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class CapBallReset extends OpMode {
    DcMotor cap_ball_lift;


    @Override
    public void init() {

            /* Initialize the hardware variables.
             * The init() method of the hardware class does all the work here
             */

        /*cap_ball_lift = hardwareMap.dcMotor.get("cap_ball_lift");
        cap_ball_tilt = hardwareMap.dcMotor.get("cap_ball_tilt");*/
        // mortar.setPower(engagePower);
        // mortar.setTargetPosition(mortarEngagedState);
        cap_ball_lift = hardwareMap.dcMotor.get("cap_ball_lift");
        cap_ball_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // cap_ball_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //cap_ball_tilt.setPower(0.3);
        //cap_ball_lift.setPower(0.5);
        cap_ball_lift.setMaxSpeed(1680);
        //cap_ball_lift.setTargetPosition(-121);
        // Wait for the game to start (driver presses PLAY)


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(gamepad1.dpad_down&&cap_ball_lift.getCurrentPosition()<0){
            cap_ball_lift.setPower(1);
        } else if(gamepad1.dpad_up&&cap_ball_lift.getCurrentPosition()>-15100){
            cap_ball_lift.setPower(-1);
        } else{
            cap_ball_lift.setPower(0);
        }
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){

    }
}


