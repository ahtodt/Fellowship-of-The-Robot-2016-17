
             package org.firstinspires.ftc.teamcode;

             import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
             import com.qualcomm.robotcore.eventloop.opmode.OpMode;
             import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
             import com.qualcomm.robotcore.hardware.DcMotor;
             import com.qualcomm.robotcore.hardware.GyroSensor;
             import com.qualcomm.robotcore.hardware.I2cAddr;
             import com.qualcomm.robotcore.hardware.Servo;

             @TeleOp(name="TeleOp", group="TeleOp")
             public class DrivingPractice extends OpMode {
                 mordorHardware robot           = new mordorHardware();
                 double left;
                 double right;
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
                 // int posCounter = 0;
                // boolean isPressed = false;
                // boolean isPressed2 = false;
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

              /*   public void posA() {
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
                 }
                 */
                 public void vibrateCam(){

                     if(robot.magazine_cam.getPosition()==camUp){
                         robot.magazine_cam.setPosition(camUp-.25);
                     }else if(robot.magazine_cam.getPosition()<=(camUp-.25)){
                         robot.magazine_cam.setPosition(camUp);
                     }
                 }
                 public void vibrateParticleGate(){
                     if(robot.collector_gate.getPosition()==PCGateDown-.2){
                         robot.collector_gate.setPosition(PCGateDown);
                     }else if(robot.collector_gate.getPosition()<=(PCGateDown)){
                         robot.collector_gate.setPosition(PCGateDown-.2);
                     }
                 }
                 public void stopMotors(){
                     robot.right_drive1.setPower(0);
                     robot.left_drive1.setPower(0);
                     robot.right_drive2.setPower(0);
                     robot.left_drive2.setPower(0);
                 }
                 public void setPowerLeft(double power){
                     robot.left_drive1.setPower(power);
                     robot.left_drive2.setPower(power);
                 }
                 public void setPowerRight(double power){
                     robot.right_drive1.setPower(power);
                     robot.right_drive2.setPower(power);
                 }
                 public void cock(){
                     if(!cocked){
                         robot.mortar.setPower(engagePower);
                         robot.mortar.setTargetPosition(mortarEngagedState);
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
                         if (robot.mortar.getCurrentPosition() < (mortarFreeState) && !waitStarted) {
                             robot.mortar.setPower(firingSpeed);
                             robot.mortar.setTargetPosition(mortarFreeState);
                         }
                         if (robot.mortar.getCurrentPosition() >= (mortarFreeState-7) && !waitFinished) {
                             robot.mortar_gate.setPosition(mortarGateUp);
                             if (!waitStarted) {

                                 resetStartTime();
                                 waitStarted = true;
                             }
                             if(waitStarted && getRuntime() > .6){
                                 robot.mortar_gate.setPosition(mortarGateDown);
                             }
                             if (waitStarted && getRuntime() > 1)
                             {
                                 waitFinished = true;
                             }
                         }

                         if (waitFinished && !encoderReset&&robot.mortar.getCurrentPosition()>=(mortarFreeState-7)) {
                             robot.mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                             robot.mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                             encoderReset = true;
                         }
                         if (encoderReset) {
                             robot.mortar.setPower(cockingSpeed);
                             robot.mortar.setTargetPosition(mortarEngagedState);
                         }
                         if (encoderReset && robot.mortar.getCurrentPosition() >= mortarEngagedState) {
                             shots++;
                             waitStarted = false;
                             waitFinished = false;
                             mortarReset = false;
                             encoderReset = false;
                             robot.mortar_gate.setPosition(mortarGateDown);

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
                     robot.init(hardwareMap);


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
         float throttle = -gamepad1.left_stick_y ;
         float direction = gamepad1.left_stick_x ;
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
         robot.left_drive1.setMaxSpeed(Math.abs(leftSpeed));
         robot.left_drive2.setMaxSpeed(Math.abs(leftSpeed));
         robot.right_drive1.setMaxSpeed(Math.abs(rightSpeed));
         robot.right_drive2.setMaxSpeed(Math.abs(rightSpeed));

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
                         robot.left_drive1.setMaxSpeed(Math.abs(leftSpeed));
                         robot.left_drive2.setMaxSpeed(Math.abs(leftSpeed));
                         robot.right_drive1.setMaxSpeed(Math.abs(rightSpeed));
                         robot.right_drive2.setMaxSpeed(Math.abs(rightSpeed));

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
                         robot.left_drive1.setMaxSpeed(Math.abs(leftSpeed));
                         robot.left_drive2.setMaxSpeed(Math.abs(leftSpeed));
                         robot.right_drive1.setMaxSpeed(Math.abs(rightSpeed));
                         robot.right_drive2.setMaxSpeed(Math.abs(rightSpeed));

                     }




     else if(precisionMode&&backwardsMode){
                         float throttle = gamepad1.left_stick_y ;
                         float direction = gamepad1.left_stick_x ;
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
         robot.left_drive1.setMaxSpeed(Math.abs(leftSpeed));
         robot.left_drive2.setMaxSpeed(Math.abs(leftSpeed));
         robot.right_drive1.setMaxSpeed(Math.abs(rightSpeed));
         robot.right_drive2.setMaxSpeed(Math.abs(rightSpeed));
                     }
                     telemetry.addData("backwards", backwardsMode);

                     if(gamepad2.right_trigger>triggerCutoff){
                         //collecting mode
                         robot.particle_collector.setPower(1);
                         robot.collector_gate.setPosition(PCGateUp);
                         robot.mortar_gate.setPosition(mortarGateUp);
                         robot.magazine_cam.setPosition(camMid);
                     }
                     else if(gamepad2.left_trigger > triggerCutoff){
                         //ejecting mode
                         robot.particle_collector.setPower(-1);
                         robot.collector_gate.setPosition(PCGateUp);
                         robot.magazine_cam.setPosition(camMid);        robot.mortar_gate.setPosition(mortarGateDown);

                     }else{
                         //drive/fire sequence
                         robot.particle_collector.setPower(0);
                         vibrateParticleGate(); // +vibrate
                         //sequenced with motor
                         vibrateCam(); // + vibrate
                     }
                     if(gamepad2.x){
                         buttonPressed = true;
                         robot.mortar_gate.setPosition(mortarGateDown);
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
                     telemetry.addData("shooterCount", shooterCount);

                     if(gamepad2.y&&robot.cap_ball_lift.getCurrentPosition()<0){
                         robot.cap_ball_lift.setPower(1);
                     } else if(gamepad1.b&&robot.cap_ball_lift.getCurrentPosition()>-15100){
                         robot.cap_ball_lift.setPower(-1);
                     } else{
                         robot.cap_ball_lift.setPower(0);
                     }

                   /*  if (gamepad2.b) {
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
                     }
                     if (gamepad2.start) {
                         posEnd();
                     }

                     */

                     if (gamepad2.left_bumper) {
                         robot.left_beacon.setPosition(.34);
                     }
                     else{
                         robot.left_beacon.setPosition(.19);
                     }

                     if (gamepad2.right_bumper) {
                         robot.right_beacon.setPosition(.41);
                     }
                     else{
                         robot.right_beacon.setPosition(.26);
                     }
                 }
                 /*
                  * Code to run ONCE after the driver hits STOP
                  */
                     @Override
                     public void stop(){

                     }
                 }


