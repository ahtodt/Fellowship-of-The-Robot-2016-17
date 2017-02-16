package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class mordorHardware
{
    /* Public OpMode members. */
    DcMotor left_drive1;
    DcMotor left_drive2;
    DcMotor right_drive1;
    DcMotor right_drive2;
    DcMotor particle_collector;
    DcMotor mortar;
    DcMotor cap_ball_lift;
    Servo collector_gate;
    Servo mortar_gate;
    Servo magazine_cam;
    Servo right_beacon;
    Servo left_beacon;
    GyroSensor gyro;
    ColorSensor floor_seeker;
    ColorSensor left_color;
    ColorSensor right_color;
    ModernRoboticsI2cRangeSensor left_range;
    I2cAddr leftRangeI2c = I2cAddr.create8bit(0x28);
    I2cAddr leftColorI2c = I2cAddr.create8bit(0x4c);
    I2cAddr floorI2c = I2cAddr.create8bit(0x70);
    I2cAddr rightColorI2c = I2cAddr.create8bit(0x3c);
    double PCGateUp = .3;
    double PCGateDown = .75;
    double mortarGateUp = .6;
    double mortarGateDown = 1;
    double camUp = 1;
    double camMid = .5;
    double camDown = .3;
    double leftBeaconOut = .34;
    double rightBeaconOut = .41;
    double leftBeaconIn = .19;
    double rightBeaconIn = .26;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public mordorHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        left_color = hwMap.colorSensor.get("left_color");
        right_color = hwMap.colorSensor.get("right_color");
        floor_seeker=hwMap.colorSensor.get("floor_seeker");
        left_color.setI2cAddress(leftColorI2c);
        right_color.setI2cAddress(rightColorI2c);
        floor_seeker.setI2cAddress(floorI2c);
        floor_seeker.enableLed(false);
        floor_seeker.enableLed(true);
        right_color.enableLed(false);
        left_color.enableLed(false);
        left_drive1=hwMap.dcMotor.get("left_drive1");
        left_drive2=hwMap.dcMotor.get("left_drive2");
        right_drive1=hwMap.dcMotor.get("right_drive1");
        right_drive2=hwMap.dcMotor.get("right_drive2");
        left_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive2.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_range = hwMap.get(ModernRoboticsI2cRangeSensor.class, "left_range");
        left_range.setI2cAddress(leftRangeI2c);
        /*cap_ball_lift = hardwareMap.dcMotor.get("cap_ball_lift");
        cap_ball_tilt = hardwareMap.dcMotor.get("cap_ball_tilt");*/
        particle_collector = hwMap.dcMotor.get("particle_collector");
        particle_collector.setDirection(DcMotorSimple.Direction.REVERSE);
        mortar = hwMap.dcMotor.get("mortar");
        mortar.setDirection(DcMotorSimple.Direction.REVERSE);
        mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // mortar.setPower(engagePower);
        // mortar.setTargetPosition(mortarEngagedState);
        cap_ball_lift = hwMap.dcMotor.get("cap_ball_lift");
        cap_ball_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_beacon = hwMap.servo.get("left_beacon");
        right_beacon=hwMap.servo.get("right_beacon");
        right_beacon.setDirection(Servo.Direction.REVERSE);
        collector_gate=hwMap.servo.get("collector_gate");
        mortar_gate=hwMap.servo.get("mortar_gate");
        magazine_cam = hwMap.servo.get("magazine_cam");
        magazine_cam.setDirection(Servo.Direction.REVERSE);
        left_beacon.setPosition(leftBeaconIn);
        right_beacon.setPosition(rightBeaconIn);
        collector_gate.setPosition(PCGateDown);
        left_range = hwMap.get(ModernRoboticsI2cRangeSensor.class, "left_range");
        mortar_gate.setPosition(mortarGateDown);
        magazine_cam.setPosition(camDown);
         gyro=hwMap.gyroSensor.get("gyro");
        // Wait for the game to start (driver presses PLAY)
        gyro.calibrate();
        while (gyro.isCalibrating()) {
        }
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

