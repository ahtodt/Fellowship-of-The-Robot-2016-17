
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class DrivingPractice extends OpMode {
    mordorHardware robot = new mordorHardware();
    double firingSpeed = .9;
    double cockingSpeed = .5;
    double engagePower = .2;
    double camPosition;
    double particleCollectorPosition;
    double mortarPosition;
    double liftPosition;
    final double triggerCutoff = .2;
    int mortarFreeState = 1440;
    int mortarEngagedState = 300;
    int shooterCount = 0;
    int shots = 0;
    boolean mortarReset = false;
    boolean buttonPressed = false;
    boolean waitStarted = false;
    boolean waitFinished = false;
    boolean encoderReset = false;
    boolean startFiring = false;
    boolean cocked = false;
    boolean backwardsMode = false;
    boolean bumperPressed = false;

    public void vibrateCam() {

        if (camPosition == robot.camUp) {
            robot.magazine_cam.setPosition(robot.camUp - .25);
        } else if (camPosition <= (robot.camUp - .25)) {
            robot.magazine_cam.setPosition(robot.camUp);
        }
    }

    public void vibrateParticleGate() {
        if (particleCollectorPosition == robot.PCGateDown - .2) {
            robot.collector_gate.setPosition(robot.PCGateDown);
        } else if (particleCollectorPosition <= (robot.PCGateDown)) {
            robot.collector_gate.setPosition(robot.PCGateDown - .2);
        }
    }

    public void cock() {
        if (!cocked) {
            robot.mortar.setPower(engagePower);
            robot.mortar.setTargetPosition(mortarEngagedState);
            cocked = true;
        }
    }

    public void shootingSequence() {
        if (shots < shooterCount) {
            if (mortarPosition < (mortarFreeState) && !waitStarted) {
                robot.mortar.setPower(firingSpeed);
                robot.mortar.setTargetPosition(mortarFreeState);
            }
            if (mortarPosition >= (mortarFreeState - 7) && !waitFinished) {
                robot.mortar_gate.setPosition(robot.mortarGateUp);
                if (!waitStarted) {

                    resetStartTime();
                    waitStarted = true;
                }
                if (waitStarted && getRuntime() > .6) {
                    robot.mortar_gate.setPosition(robot.mortarGateDown);
                }
                if (waitStarted && getRuntime() > 1) {
                    waitFinished = true;
                }
            }

            if (waitFinished && !encoderReset && mortarPosition >= (mortarFreeState - 7)) {
                robot.mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoderReset = true;
            }
            if (encoderReset) {
                robot.mortar.setPower(cockingSpeed);
                robot.mortar.setTargetPosition(mortarEngagedState);
            }
            if (encoderReset && mortarPosition >= mortarEngagedState) {
                shots++;
                waitStarted = false;
                waitFinished = false;
                mortarReset = false;
                encoderReset = false;
                robot.mortar_gate.setPosition(robot.mortarGateDown);

            }
        }
        if (shots == shooterCount) {
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
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        camPosition = robot.magazine_cam.getPosition();
        particleCollectorPosition = robot.collector_gate.getPosition();
        mortarPosition = robot.mortar.getCurrentPosition();
        liftPosition = robot.cap_ball_lift.getCurrentPosition();
        float throttle = gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;

        if (gamepad1.right_bumper) {
            bumperPressed = true;
        }
        if (bumperPressed && !gamepad1.right_bumper) {
            if (backwardsMode) {
                backwardsMode = false;
            } else {
                backwardsMode = true;
            }
            bumperPressed = false;
        }


        /*
        driving code
         */
        if (!backwardsMode) {
            double right = -(throttle*1.2) - (direction*.7);
            double left = -(throttle*1.2) + (direction*.7);

            int leftSpeed = (int) (1300 * left);
            int rightSpeed = (int) (1300 * right);

            if (leftSpeed > 0) {
                robot.setPowerLeft(1);
            } else if (leftSpeed < 0) {
                robot.setPowerLeft(-1);
            }
            if (rightSpeed > 0) {
                robot.setPowerRight(1);
            } else if (rightSpeed < 0) {
                robot.setPowerRight(-1);
            }
            robot.left_drive1.setMaxSpeed(Math.abs(leftSpeed));
            robot.left_drive2.setMaxSpeed(Math.abs(leftSpeed));
            robot.right_drive1.setMaxSpeed(Math.abs(rightSpeed));
            robot.right_drive2.setMaxSpeed(Math.abs(rightSpeed));

        } else if (backwardsMode) {
            double right = (throttle*1.2) - (direction*.7);
            double left = (throttle*1.2) + (direction*.7);
            int leftSpeed = (int) (1300 * left);
            int rightSpeed = (int) (1300 * right);

            if (leftSpeed > 0) {
                robot.setPowerLeft(1);
            } else if (leftSpeed < 0) {
                robot.setPowerLeft(-1);
            }
            if (rightSpeed > 0) {
                robot.setPowerRight(1);
            } else if (rightSpeed < 0) {
                robot.setPowerRight(-1);
            }
            robot.left_drive1.setMaxSpeed(Math.abs(leftSpeed));
            robot.left_drive2.setMaxSpeed(Math.abs(leftSpeed));
            robot.right_drive1.setMaxSpeed(Math.abs(rightSpeed));
            robot.right_drive2.setMaxSpeed(Math.abs(rightSpeed));
        }
        telemetry.addData("backwards", backwardsMode);



        /*
        code for gate sytem
        */
        if (gamepad2.right_trigger > triggerCutoff) {
            //collecting mode
            robot.particle_collector.setPower(1);
            robot.collector_gate.setPosition(robot.PCGateUp);
            robot.mortar_gate.setPosition(robot.mortarGateUp);
            robot.magazine_cam.setPosition(robot.camMid);
        } else if (gamepad2.left_trigger > triggerCutoff) {
            //ejecting mode
            robot.particle_collector.setPower(-1);
            robot.collector_gate.setPosition(robot.PCGateUp);
            vibrateCam();
            robot.mortar_gate.setPosition(robot.mortarGateDown);

        } else {
            //drive/fire sequence
            robot.particle_collector.setPower(0);
            vibrateParticleGate(); // +vibrate
            //sequenced with motor
            vibrateCam(); // + vibrate
        }



                     /*
                     mortar shooting controls
                      */
        if (gamepad2.x) {
            buttonPressed = true;
            robot.mortar_gate.setPosition(robot.mortarGateDown);
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



                     /*
                     cap ball code
                      */
        if (gamepad2.dpad_down && liftPosition < 0) {
            robot.cap_ball_lift.setPower(1);
        } else if (gamepad2.dpad_up && liftPosition > -15100) {
            robot.cap_ball_lift.setPower(-1);
        } else {
            robot.cap_ball_lift.setPower(0);
        }



                     /*
                     beacon arm code
                      */
        if (gamepad2.left_bumper) {
            robot.left_beacon.setPosition(robot.leftBeaconOut);
        } else {
            robot.left_beacon.setPosition(robot.leftBeaconIn);
        }

        if (gamepad2.right_bumper) {
            robot.right_beacon.setPosition(robot.rightBeaconOut);
        } else {
            robot.right_beacon.setPosition(robot.rightBeaconIn);
        }
    }
}


