

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
    DcMotor left_drive2;
    DcMotor right_drive1;
    DcMotor right_drive2;
    DcMotor particle_collector;
    DcMotor mortar;
    DcMotor cap_ball_tilt;
    DcMotor cap_ball_lift;
    Servo collector_gate;
    Servo mortar_gate;
    Servo magazine_cam;
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
        left_drive2=hardwareMap.dcMotor.get("left_drive2");
        right_drive1=hardwareMap.dcMotor.get("right_drive1");
        right_drive2=hardwareMap.dcMotor.get("right_drive2");
        left_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        cap_ball_lift = hardwareMap.dcMotor.get("cap_ball_lift");
        cap_ball_tilt = hardwareMap.dcMotor.get("cap_ball_tilt");
        particle_collector = hardwareMap.dcMotor.get("particle_collector");
        mortar = hardwareMap.dcMotor.get("mortar");
        left_beacon=hardwareMap.servo.get("left_beacon");
        right_beacon=hardwareMap.servo.get("right_beacon");
        collector_gate=hardwareMap.servo.get("collector_gate");
        mortar_gate=hardwareMap.servo.get("mortar_gate");
        magazine_cam = hardwareMap.servo.get("magazine_cam");
        left_beacon.setPosition(0);
        right_beacon.setPosition(0);
        collector_gate.setPosition(0);
        mortar_gate.setPosition(0);
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

        if(!gamepad1.x){
            leftReset = false;
        }
        if(!gamepad1.b){
            rightReset=false;
        }


        if(gamepad1.y&&!liftReset){
            if(!lifting){
                cap_ball_lift.setPower(0.5);
                lifting = true;
            }else{
                cap_ball_lift.setPower(0);
                lifting = false;
            }
            liftReset = true;
        }
        if(!gamepad1.y){
            liftReset = false;
        }

        if(gamepad1.a&&!mortarReset){
            if(!firing){
                mortar.setPower(0.5);
                firing = true;
            }else{
                mortar.setPower(0);
                firing = false;
            }
            mortarReset = true;
        }
        if(!gamepad1.a){
            mortarReset = false;
        }

        if(gamepad1.right_bumper&&!particleCollectorReset){
            if(!collecting){
                particle_collector.setPower(0.5);
                collecting = true;
            }else{
                particle_collector.setPower(0);
                collecting = false;
            }
            particleCollectorReset = true;
        }
        if(!gamepad1.right_bumper){
            particleCollectorReset = false;
        }


    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

