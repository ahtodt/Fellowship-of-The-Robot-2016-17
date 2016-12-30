

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


public class FirstTele extends OpMode {
    double left;
    double right;
    //DcMotor left_drive1;
    //DcMotor left_drive2;
    //DcMotor right_drive1;
    //DcMotor right_drive2;
    DcMotor particle_collector;
    DcMotor mortar;
    DcMotor cap_ball_tilt;
    DcMotor cap_ball_lift;
    Servo collector_gate;
     Servo mortar_gate;
     Servo magazine_cam;
     Servo right_beacon;
     Servo left_beacon;
     double baselinePower = .2;
     double powerCoefficient = .0001;
     double mortarPower;
     boolean RightDown = false;
     boolean LeftDown = false;
     boolean rightReset = false;
     boolean leftReset = false;
     boolean lifting = false;
    boolean liftReset = false;
    boolean posA = false;
    boolean posB = false;
    boolean posC = false;
    boolean posD = false;
    boolean posE = false;
    boolean firing = false;
    boolean mortarReset = false;
    boolean collecting = false;
    boolean particleCollectorReset = false;

    // GyroSensor gyro;

   public void shoot() {
       mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       mortar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       mortar.setPower(.3);
       mortar.setTargetPosition(1440);
       mortar.setPower(0);

       mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       mortar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

}

    public void posA() {
        // drive
        // drop cam before slide goes down
        posA = true;
        cap_ball_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        cap_ball_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cap_ball_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        cap_ball_tilt.setPower(0.3);
        cap_ball_tilt.setTargetPosition(80);


    }

    public void posB() {
        // scoop
        posB = true;
        cap_ball_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        cap_ball_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cap_ball_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        cap_ball_tilt.setPower(0.3);
        cap_ball_tilt.setTargetPosition(400);
    }

    public void posC(){
        // hold
        posC = true;
        cap_ball_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        cap_ball_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cap_ball_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        cap_ball_tilt.setPower(0.3);
        cap_ball_tilt.setTargetPosition(300);
    }

    public void posD(){
        // raised
        posD = true;
        cap_ball_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        cap_ball_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cap_ball_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        cap_ball_tilt.setPower(0.3);
        cap_ball_tilt.setTargetPosition(350);
    }

    public void posE(){
        // drop
        cap_ball_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        cap_ball_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cap_ball_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        cap_ball_tilt.setPower(0.3);
        cap_ball_tilt.setTargetPosition(480);
    }

    public void posReset(){
        posA = false;
        posB = false;
        posC = false;
        posD = false;
        posE = false;
    }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

    /* Initialize the hardware variables.
     * The init() method of the hardware class does all the work here
     */

        //left_drive1=hardwareMap.dcMotor.get("left_drive1");
        //left_drive2=hardwareMap.dcMotor.get("left_drive2");
        //right_drive1=hardwareMap.dcMotor.get("right_drive1");
        //right_drive2=hardwareMap.dcMotor.get("right_drive2");
        //left_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
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
        mortar.setPower(1);
        // telemetry.addData("gyro", gyro.getHeading());
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        //left_drive1.setPower(left);
        //left_drive2.setPower(left);
        //right_drive1.setPower(right);
        //right_drive2.setPower(right);

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

        if(gamepad2.b){
            if(posA){
                posA = true;
            } else {
                posA();
            }
            if(posB){
                posB = true;
            } else {
                posB();
            }
            if(posC){
                posC = true;
            } else {
                posC();
            }
            if(posD){
                posD = true;
            } else {
                posD();
            }
            if(posE){
                posE = true;
            } else {
                posE();
            }

        }
        if(gamepad1.a&&!mortarReset){
            mortarReset = true;
            shoot();

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
        public void stop () {
        }

    }

