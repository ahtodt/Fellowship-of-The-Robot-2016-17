

        package org.firstinspires.ftc.robotcontroller.external.samples;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.GyroSensor;


        public class DrivingPractice extends OpMode {
            double left;
            double right;
            DcMotor left_drive1;
            DcMotor left_drive2;
            DcMotor right_drive1;
            DcMotor right_drive2;
            //DcMotor particle_collector;
            DcMotor mortar;
            /*DcMotor cap_ball_tilt;
            DcMotor cap_ball_lift;
            Servo collector_gate;
            Servo mortar_gate;
            Servo magazine_cam;
            Servo right_beacon;
            Servo left_beacon;  */
            double baselinePower = .2;
            double powerCoefficient = .0001;
            double mortarPower;
            double firingSpeed = .9;
            double cockingSpeed = .5;
            double engagePower =.2;
            int mortarFreeState;
            int mortarEngagedState = 300;
            int mortarReadyState;
            int lastError=0;
            int shooterCount = 0;
            int shots = 0;
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

            GyroSensor gyro;

            /*
             * Code to run ONCE when the driver hits INIT
             */
            @Override
            public void init() {

            /* Initialize the hardware variables.
             * The init() method of the hardware class does all the work here
             */

                left_drive1 = hardwareMap.dcMotor.get("left_drive1");
                left_drive2 = hardwareMap.dcMotor.get("left_drive2");
                right_drive1 = hardwareMap.dcMotor.get("right_drive1");
                right_drive2 = hardwareMap.dcMotor.get("right_drive2");
                left_drive2.setDirection(DcMotorSimple.Direction.REVERSE);
                right_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
                /*cap_ball_lift = hardwareMap.dcMotor.get("cap_ball_lift");
                cap_ball_tilt = hardwareMap.dcMotor.get("cap_ball_tilt");*/
                //particle_collector = hardwareMap.dcMotor.get("particle_collector");
               // particle_collector.setDirection(DcMotorSimple.Direction.REVERSE);
                mortar = hardwareMap.dcMotor.get("mortar");
                mortar.setDirection(DcMotorSimple.Direction.REVERSE);
                mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mortar.setPower(engagePower);
                mortar.setTargetPosition(mortarEngagedState);
                /*left_beacon=hardwareMap.servo.get("left_beacon");
                right_beacon=hardwareMap.servo.get("right_beacon");
                collector_gate=hardwareMap.servo.get("collector_gate");
                mortar_gate=hardwareMap.servo.get("mortar_gate");
                magazine_cam = hardwareMap.servo.get("magazine_cam");
                left_beacon.setPosition(0);
                right_beacon.setPosition(0);
                collector_gate.setPosition(0);
                mortar_gate.setPosition(0);
                gyro=hardwareMap.gyroSensor.get("gyro");
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
                left_drive1.setPower(left * 0.5);
                left_drive2.setPower(left * 0.5);
                right_drive1.setPower(right * 0.5);
                right_drive2.setPower(right * 0.5);
                telemetry.addData("mortar", mortar.getCurrentPosition());
                telemetry.addData("shooterCount", shooterCount);
                telemetry.addData("runtime", getRuntime());
                if(gamepad2.a){
                    mortar.setPower(1);
                }mortar.setPower(0);

                //cap ball mechanism on gamepad 2 stick

                /*if(gamepad1.x&&!leftReset){
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

                /*if(gamepad1.right_bumper&&!particleCollectorReset){
                    if(!collecting){
                        particle_collector.setPower(1);
                        collecting = true;
                    }else{
                        particle_collector.setPower(0);
                        collecting = false;
                    }
                    particleCollectorReset = true;
                }
                if(!gamepad1.right_bumper){
                    particleCollectorReset = false;
                }*/


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
                        if (waitStarted && (getRuntime() > 1)) ;
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
             * Code to run ONCE after the driver hits STOP
             */
                @Override
                public void stop() {
                }

            }


