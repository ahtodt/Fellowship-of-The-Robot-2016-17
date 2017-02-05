

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
            boolean backwardsMode = false;
            boolean bumperPressed = false;
            boolean precisionMode = true;
            boolean bumperPrecision = false;
            boolean bumperPressed2 =false;
            boolean capBallTurn = false;
            GyroSensor gyro;

            public void posA() {
                // drive
                // drop cam before slide goes down
                cap_ball_tilt.setPower(0.3);
                cap_ball_tilt.setTargetPosition(40);
            }

            public void posB() {
                // scoop
                cap_ball_lift.setPower(0.5);
                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(-121);
                cap_ball_tilt.setPower(0.3);
                cap_ball_tilt.setTargetPosition(370);
            }

            public void posC(){
                // hold
                cap_ball_tilt.setPower(0);
                cap_ball_lift.setPower(0.5);
                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(-2714);
            }

            public void posD(){
                // raised
                cap_ball_lift.setPower(1);


                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(-14934);
            }

            public void posE(){
                // drop
                cap_ball_lift.setPower(1);
                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(-10722);
            }

            public void posA2(){
                cap_ball_lift.setPower(1);
                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(-121);
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
                telemetry.addData("Position", posCounter);
            }
            public void vibrateCam(){

                if(magazine_cam.getPosition()==camUp){
                    magazine_cam.setPosition(camUp-.25);
                }else if(magazine_cam.getPosition()<=(camUp-.25)){
                    magazine_cam.setPosition(camUp);
                }
            }
            public void vibrateParticleGate(){
                if(collector_gate.getPosition()==PCGateDown-.2){
                    collector_gate.setPosition(PCGateDown);
                }else if(collector_gate.getPosition()<=(PCGateDown)){
                    collector_gate.setPosition(PCGateDown-.2);
                }
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
                    mortarFreeState = 1440;
                }else {
                    mortarFreeState = 1440;
                }
                if(shots<shooterCount) {
                    if (mortar.getCurrentPosition() < (mortarFreeState) && !waitStarted) {
                        mortar.setPower(firingSpeed);
                        mortar.setTargetPosition(mortarFreeState);
                    }
                    if (mortar.getCurrentPosition() >= (mortarFreeState-7) && !waitFinished) {
                        mortar_gate.setPosition(mortarGateUp);
                        if (!waitStarted) {

                            resetStartTime();
                            waitStarted = true;
                        }
                        if(waitStarted && getRuntime() > .6){
                            mortar_gate.setPosition(mortarGateDown);
                        }
                        if (waitStarted && getRuntime() > 1)
                        {
                            waitFinished = true;
                        }
                    }

                    if (waitFinished && !encoderReset&&mortar.getCurrentPosition()>=(mortarFreeState-7)) {
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
                        mortar_gate.setPosition(mortarGateDown);

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
                right_drive2.setDirection(DcMotorSimple.Direction.REVERSE);
                left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                cap_ball_tilt.setPower(0.3);
                cap_ball_lift.setPower(0.5);
                cap_ball_lift.setMaxSpeed(1680);
                cap_ball_lift.setTargetPosition(-121);
                left_beacon=hardwareMap.servo.get("left_beacon");
                right_beacon=hardwareMap.servo.get("right_beacon");
                right_beacon.setDirection(Servo.Direction.REVERSE);
                collector_gate=hardwareMap.servo.get("collector_gate");
                mortar_gate=hardwareMap.servo.get("mortar_gate");
                magazine_cam = hardwareMap.servo.get("magazine_cam");
                magazine_cam.setDirection(Servo.Direction.REVERSE);
                left_beacon.setPosition(0.19);
                right_beacon.setPosition(0.26);
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

                if(gamepad1.right_bumper){
                    bumperPressed = true;
                }
                if(bumperPressed && !gamepad1.right_bumper){
                    if(backwardsMode){
                        backwardsMode = false;
                    }else{
                        backwardsMode = true;
                    }
                    bumperPressed = false;

                }

                if(gamepad1.left_bumper){
                    bumperPressed2 = true;
                }
                if(bumperPressed2 && !gamepad1.left_bumper){
                    if(capBallTurn){
                        capBallTurn = false;
                    }else{
                        capBallTurn = true;
                    }
                    bumperPressed2 = false;

                }




if(precisionMode&&!backwardsMode) {
    float throttle = -gamepad1.left_stick_y;
    float direction = gamepad1.right_stick_x;
    float right = throttle - direction;
    float left = throttle + direction;

    int leftSpeed = (int) (800 * left);
    int rightSpeed = (int) (800 * right);

    if (leftSpeed > 0) {
        setPowerLeft(1);
    } else if (leftSpeed < 0) {
        setPowerLeft(-1);
    }
    if (rightSpeed > 0) {
        setPowerRight(1);
    } else if (rightSpeed < 0) {
        setPowerRight(-1);
    }
    left_drive1.setMaxSpeed(Math.abs(leftSpeed));
    left_drive2.setMaxSpeed(Math.abs(leftSpeed));
    right_drive1.setMaxSpeed(Math.abs(rightSpeed));
    right_drive2.setMaxSpeed(Math.abs(rightSpeed));

}else if(capBallTurn&&!backwardsMode){
                    float throttle = -gamepad1.left_stick_y ;
                    float direction = gamepad1.right_stick_x ;
                    float right = throttle - direction;
                    float left = throttle + direction;

                    int leftSpeed = (int) (400 * left);
                    int rightSpeed = (int) (400 * right);

                    if (leftSpeed > 0) {
                        setPowerLeft(1);
                    } else if (leftSpeed < 0) {
                        setPowerLeft(-1);
                    }
                    if (rightSpeed > 0) {
                        setPowerRight(1);
                    } else if (rightSpeed < 0) {
                        setPowerRight(-1);
                    }
                    left_drive1.setMaxSpeed(Math.abs(leftSpeed));
                    left_drive2.setMaxSpeed(Math.abs(leftSpeed));
                    right_drive1.setMaxSpeed(Math.abs(rightSpeed));
                    right_drive2.setMaxSpeed(Math.abs(rightSpeed));

                }else
                if(capBallTurn&&backwardsMode){
                    float throttle = gamepad1.left_stick_y ;
                    float direction = gamepad1.right_stick_x ;
                    float right = throttle - direction;
                    float left = throttle + direction;

                    int leftSpeed = (int) (400 * left);
                    int rightSpeed = (int) (400 * right);

                    if (leftSpeed > 0) {
                        setPowerLeft(1);
                    } else if (leftSpeed < 0) {
                        setPowerLeft(-1);
                    }
                    if (rightSpeed > 0) {
                        setPowerRight(1);
                    } else if (rightSpeed < 0) {
                        setPowerRight(-1);
                    }
                    left_drive1.setMaxSpeed(Math.abs(leftSpeed));
                    left_drive2.setMaxSpeed(Math.abs(leftSpeed));
                    right_drive1.setMaxSpeed(Math.abs(rightSpeed));
                    right_drive2.setMaxSpeed(Math.abs(rightSpeed));

                }




else if(precisionMode&&backwardsMode){
                    float throttle = gamepad1.left_stick_y ;
                    float direction = gamepad1.right_stick_x ;
                    float right = throttle - direction;
                    float left = throttle + direction;
    int leftSpeed = (int) (800 * left);
    int rightSpeed = (int) (800 * right);

    if (leftSpeed > 0) {
        setPowerLeft(1);
    } else if (leftSpeed < 0) {
        setPowerLeft(-1);
    }
    if (rightSpeed > 0) {
        setPowerRight(1);
    } else if (rightSpeed < 0) {
        setPowerRight(-1);
    }
    left_drive1.setMaxSpeed(Math.abs(leftSpeed));
    left_drive2.setMaxSpeed(Math.abs(leftSpeed));
    right_drive1.setMaxSpeed(Math.abs(rightSpeed));
    right_drive2.setMaxSpeed(Math.abs(rightSpeed));
                }
telemetry.addData("precicison", precisionMode);
                telemetry.addData("backwards", backwardsMode);

                if(gamepad2.right_trigger>triggerCutoff){
                    //collecting mode
                    particle_collector.setPower(1);
                    collector_gate.setPosition(PCGateUp);
                    mortar_gate.setPosition(mortarGateUp);
                    magazine_cam.setPosition(camMid);
                }
                else if(gamepad2.left_trigger > triggerCutoff){
                    //ejecting mode
                    particle_collector.setPower(-1);
                    collector_gate.setPosition(PCGateUp);
                    magazine_cam.setPosition(camMid);        mortar_gate.setPosition(mortarGateDown);

                }else{
                    //drive/fire sequence
                    particle_collector.setPower(0);
                    vibrateParticleGate(); // +vibrate
                    //sequenced with motor
                    vibrateCam(); // + vibrate
                }
                if(gamepad2.x){
                    buttonPressed = true;
                    mortar_gate.setPosition(mortarGateDown);
                }
                if (buttonPressed && !gamepad2.x) {
                    shooterCount++;
                    buttonPressed = false;
                    cock();
                }
                if (gamepad2.a && !mortarReset) {

                    startFiring = true;
                }
                if (startFiring) {
                    shootingSequence();
                }
                telemetry.addData("mortar", mortar.getCurrentPosition());
                telemetry.addData("shooterCount", shooterCount);
                telemetry.addData("runtime", getRuntime());

                if (gamepad2.b) {
                    isPressed = true;
                }
                if (isPressed && !gamepad2.b) {
                    isPressed = false;
                    posCounter++;
                    posCheck();
                    if (posCounter > 7) {
                        posCounter = 0;
                    }
                }

                if (gamepad2.y) {
                    isPressed2 = true;
                }
                if (isPressed2 && !gamepad2.y) {
                    isPressed2 = false;
                    posCounter--;
                    posCheck();
                    if (posCounter < -1) {
                        posCounter = 0;
                    }
                }
                /*if(shots<1){
                    magazine_cam.setPosition(camDown);
                }*/
                if (gamepad2.start) {
                    posEnd();
                }

                if (gamepad2.left_bumper) {
                    left_beacon.setPosition(.34);
                }
                else{
                    left_beacon.setPosition(.19);
                }

                if (gamepad2.right_bumper) {
                    right_beacon.setPosition(.41);
                }
                else{
                    right_beacon.setPosition(.26);
                }
            }
            /*
             * Code to run ONCE after the driver hits STOP
             */
                @Override
                public void stop(){

                }
            }


