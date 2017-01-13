

        package org.firstinspires.ftc.robotcontroller.external.samples;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.GyroSensor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.Range;


        public class DrivingPractice extends OpMode {
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
            double baselinePower = .2;
            double powerCoefficient = .0001;
            double mortarPower;
            double firingSpeed = .9;
            double cockingSpeed = .5;
            double engagePower =.2;
            final double triggerCutoff = .2;
            double PCGateUp = .3;
            double PCGateDown = .75;
            double mortarGateUp = .6;
            double mortarGateDown = 1;
            double camUp = 1;
            double camMid = .5;
            double camDown = .3;
            int mortarFreeState;
            int mortarEngagedState = 300;
            int mortarReadyState;
            int lastError=0;
            int shooterCount = 0;
            int shots = 0;
            int posCounter = 0;
            boolean isPressed = false;
            boolean isPressed2 = false;
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
            boolean poop = false;
            boolean buttonPressed = false;
            boolean waitStarted = false;
            boolean wait2Started = false;
            boolean wait2finished =false;
            boolean waitFinished = false;
            boolean encoderReset = false;
            boolean startFiring = false;
            boolean cocked = false;
            GyroSensor gyro;

            public void posA() {
                // drive
                // drop cam before slide goes down
                cap_ball_tilt.setPower(0.3);
                cap_ball_tilt.setTargetPosition(40);
            }

            public void posB() {
                // scoop
                cap_ball_tilt.setPower(0.3);
                cap_ball_tilt.setTargetPosition(370);
            }

            public void posC(){
                // hold
                cap_ball_tilt.setPower(0);
                cap_ball_lift.setPower(0.5);
                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(2682);
            }

            public void posD(){
                // raised
                cap_ball_lift.setPower(1);
                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(14760);
            }

            public void posE(){
                // drop
                cap_ball_lift.setPower(1);
                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(9750);
            }

            public void posA2(){
                cap_ball_lift.setPower(0.5);
                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(120);
            }

            public void posEnd(){

                posCounter = 0;

                cap_ball_lift.setPower(0.5);
                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(0);
                cap_ball_tilt.setPower(0.2);
                cap_ball_tilt.setTargetPosition(0);

            }

            public void posReset(){
                // posCounter set one below because of incrementing when button pressed
                posCounter = 0;
                posCheck();
            }

            public void posCheck(){
                if(posCounter == 0){
                    posA2();
                }
                if(posCounter == 1){
                    posA();
                }
                if(posCounter == 2){
                    posB();
                }
                if(posCounter == 3){
                    posC();
                }
                if(posCounter == 4){
                    posD();
                }
                if(posCounter == 5){
                    posE();
                }
                if(posCounter == 6) {
                    posReset();
                }
                telemetry.addData("Position",posCounter);
            }

            public void stopMotors(){
                right_drive1.setPower(0);
                left_drive1.setPower(0);
                right_drive2.setPower(0);
                left_drive2.setPower(0);
            }
            public void setPowerLeft(double power){
                left_drive1.setPower(power);
                left_drive2.setPower(power);
            }
            public void setPowerRight(double power){
                right_drive1.setPower(power);
                right_drive2.setPower(power);
            }
            public void cock(){
                if(!cocked){
                    mortar.setPower(engagePower);
                    mortar.setTargetPosition(mortarEngagedState);
                    cocked = true;
                }
            }
            public void shootingSequence(){
                if(shots>1){
                    mortarFreeState = 1305;
                    mortarReadyState = 1305;
                }else{
                    mortarFreeState = 1290;
                    mortarReadyState = 1290;
                }
                if(shots<shooterCount) {
                    if (mortar.getCurrentPosition() < mortarFreeState && !waitStarted) {
                        mortar.setPower(firingSpeed);
                        mortar.setTargetPosition(mortarFreeState);
                    }
                    if (mortar.getCurrentPosition() >= mortarFreeState && !waitFinished) {
                        if (!waitStarted) {
                            resetStartTime();
                            waitStarted = true;
                        }
                        if (waitStarted && getRuntime() > 1);
                        {

                            waitFinished = true;
                        }
                    }
                    if(waitFinished){
                        mortar.setTargetPosition(mortarReadyState);
                    }
                    if (waitFinished && !encoderReset&&mortar.getCurrentPosition()>=mortarFreeState) {
                        mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        encoderReset = true;
                    }
                    if (encoderReset) {
                        mortar.setPower(cockingSpeed);
                        mortar.setTargetPosition(mortarEngagedState);
                    }
                    if (encoderReset && mortar.getCurrentPosition() >= mortarEngagedState) {
                        shots++;
                        waitStarted = false;
                        waitFinished = false;
                        mortarReset = false;
                        encoderReset = false;

                    }
                }
                if(shots==shooterCount){
                    shooterCount = 0;
                    shots = 0;
                    startFiring = false;
                }

            }
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
        /*cap_ball_lift = hardwareMap.dcMotor.get("cap_ball_lift");
        cap_ball_tilt = hardwareMap.dcMotor.get("cap_ball_tilt");*/
                particle_collector = hardwareMap.dcMotor.get("particle_collector");
                particle_collector.setDirection(DcMotorSimple.Direction.REVERSE);
                mortar = hardwareMap.dcMotor.get("mortar");
                mortar.setDirection(DcMotorSimple.Direction.REVERSE);
                mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // mortar.setPower(engagePower);
                // mortar.setTargetPosition(mortarEngagedState);
                cap_ball_lift = hardwareMap.dcMotor.get("cap_ball_lift");
                cap_ball_tilt = hardwareMap.dcMotor.get("cap_ball_tilt");
                cap_ball_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cap_ball_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cap_ball_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cap_ball_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cap_ball_lift.setPower(0.5);
                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(120);
                left_beacon=hardwareMap.servo.get("left_beacon");
                right_beacon=hardwareMap.servo.get("right_beacon");
                right_beacon.setDirection(Servo.Direction.REVERSE);
                collector_gate=hardwareMap.servo.get("collector_gate");
                mortar_gate=hardwareMap.servo.get("mortar_gate");
                magazine_cam = hardwareMap.servo.get("magazine_cam");
                magazine_cam.setDirection(Servo.Direction.REVERSE);
                left_beacon.setPosition(0.2);
                right_beacon.setPosition(0.3);
                collector_gate.setPosition(PCGateDown);
/*        gyro=hardwareMap.gyroSensor.get("gyro");
        // Wait for the game to start (driver presses PLAY)
        gyro.calibrate(); */
                mortar_gate.setPosition(mortarGateDown);
                magazine_cam.setPosition(camDown);
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
                float throttle = -gamepad1.left_stick_y;
                float direction = gamepad1.left_stick_x;
                float right = throttle - direction;
                float left = throttle + direction;
                right = Range.clip(right, -1, 1);
                left = Range.clip(left, -1, 1);

                setPowerLeft(left);
                setPowerRight(right);

                if(gamepad2.x){
                    buttonPressed = true;
                    mortar_gate.setPosition(mortarGateDown);
                }
                if(buttonPressed&&!gamepad2.x){
                    shooterCount++;
                    buttonPressed = false;
                    cock();
                }
                if(gamepad2.a&&!mortarReset){

                    startFiring = true;
                }
                if(startFiring){
                    shootingSequence();
                }
                telemetry.addData("mortar", mortar.getCurrentPosition());
                telemetry.addData("shooterCount", shooterCount);
                telemetry.addData("runtime", getRuntime());

                if(gamepad2.b){
                    isPressed = true;
                }
                if(isPressed&&!gamepad2.b){
                    isPressed = false;
                    posCheck();
                    posCounter ++;
                    if(posCounter > 7){
                        posCounter = 0;
                    }
                }

               /* if(gamepad2.x){
                    isPressed2 = true;
                }
                if(isPressed2&&!gamepad2.x){
                    isPressed2 = false;
                    posCheck();
                    posCounter --;
                    if(posCounter < -1){
                        posCounter = 0;
                    }
                } */

                if(gamepad2.start){
                    posEnd();
                }

            }





            /*
             * Code to run ONCE after the driver hits STOP
             */
                @Override
                public void stop() {
                }

            }


