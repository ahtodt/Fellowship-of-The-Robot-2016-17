

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
    DcMotor left_drive1;
    DcMotor left_drive2
    DcMotor right_drive1;
    DcMotor right_drive2;
    DcMotor particle_collector:
    DcMotor mortar;
    DcMotor cap_ball_tilt;
    DcMotor cap_ball_lift
    Servo particle_collector1;
    Servo particle_collector2;
    Servo right_beacon;
    Servo left_beacon;
    boolean RightDown = false;
    boolean LeftDown = false;
    boolean rightReset = false;
    boolean leftReset = false;
    boolean lifting = false;
    boolean liftReset = false;
    boolean firing = false;
    boolean mortarReset = false;
    boolean collecting = false;
    boolean particleCollectorReset = false;
   // GyroSensor gyro;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

    /* Initialize the hardware variables.
     * The init() method of the hardware class does all the work here
     */



        left_drive1=hardwareMap.dcMotor.get("left_drive1");
        right_drive1=hardwareMap.dcMotor.get("right_drive1");
        left_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        left_beacon=hardwareMap.servo.get("left_beacon");
        right_beacon=hardwareMap.servo.get("right_beacon");
        particle_collector1=hardwareMap.servo.get("particle_collector1");
        particle_collector2=hardwareMap.servo.get("particle_collector2");
        left_beacon.setPosition(0);
        right_beacon.setPosition(0);
        particle_collector1.setPosition(0);
        particle_collector2.setPosition(0);
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
        left_drive1.setPower(left);
        left_drive2.setPower(left);
        right_drive1.setPower(right);
        right_drive2.setPower(right);

        //cap ball mechanism on gamepad 2 stick

        if(gamepad1.x&&!leftReset){
            if(!LeftDown){
                left_beacon.setPosition((1));
                LeftDown = true;
            }else{
                left_beacon.setPosition(- 1);
                LeftDown = false;
            }
            leftReset = true;
        }

        if(gamepad1.b&&!rightReset){
            if(!RightDown){
                right_beacon.setPosition((1));
                RightDown = true;
            }else{
                right_beacon.setPosition(-1);
                RightDown = false;
            }
            rightReset=true;
            
        }


        if(gamepad2.a&&!liftReset){
            if(!lifting){
                cap_ball_lift.setPower(0.5);
                lifting = true;
            }
            liftReset = true;
        }

        if(gamepad2.y&&!mortarReset){
            if(!firing){
                mortar.setPower(0.5);
                firing = true;
            }
            mortarReset = true;
        }

        if(gamepad1.y&&!particleCollectorReset){
            if(!collecting){
                particle_collector.setPower(0.5);
                collecting = true;
            }
            particleCollectorReset = true
        }

        if(!gamepad1.x){
            leftReset = false;
        }
        if(!gamepad1.b){
            rightReset=false;
        }

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

