

        package org.firstinspires.ftc.robotcontroller.external.samples;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.GyroSensor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.robot.Robot;
        import com.qualcomm.robotcore.util.Range;
        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;


public class FirstTele extends OpMode{
    double left;
    double right;
    DcMotor leftMotor;
    DcMotor rightMotor;
    Servo rightServo;
    Servo leftServo;
    boolean RightDown = false;
    boolean LeftDown = false;
   // GyroSensor gyro;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

    /* Initialize the hardware variables.
     * The init() method of the hardware class does all the work here
     */



        leftMotor=hardwareMap.dcMotor.get("left_drive");
        rightMotor=hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftServo=hardwareMap.servo.get("left_servo");
        rightServo=hardwareMap.servo.get("right_servo");
/*        gyro=hardwareMap.gyroSensor.get("gyro");
        // Wait for the game to start (driver presses PLAY)
        gyro.calibrate(); */

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
       // telemetry.addData("gyro", gyro.getHeading());
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        leftMotor.setPower(left);
        rightMotor.setPower(right);

        if(gamepad1.dpad_left){
            if(!LeftDown){
                leftServo.setPosition((1));
                LeftDown = true;
            }else{
                leftServo.setPosition(1);
                LeftDown = false;
            }
        }

        if(gamepad1.dpad_right){
            if(!RightDown){
                rightServo.setPosition((1));
                RightDown = true;
            }else{
                rightServo.setPosition(1);
                RightDown = false;
            }
        }
    }




    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

