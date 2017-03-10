package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorAdafruitRGB;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is Mordor
 *
 * Everything here is used in both auto and tele
 *
 * don't fuck this up
 *
 */
public class mordorHardware {
    /* Public OpMode members. */
    DeviceInterfaceModule leftDim;
    DeviceInterfaceModule rightDim;
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
    ColorSensor floor_seeker;
    ColorSensor frontColor;
    I2cAddr floorI2c = I2cAddr.create8bit(0x70);
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    final double PCGateUp = .3;
    final double PCGateDown = .75;
    final double mortarGateUp = .6;
    final double mortarGateDown = 1;
    final double camUp = 1;
    final double camMid = .5;
    final double camDown = .3;
    final double leftBeaconOut = .34;
    final double rightBeaconOut = .41;
    final double leftBeaconIn = .19;
    final double rightBeaconIn = .26;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public mordorHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDim = hwMap.deviceInterfaceModule.get("leftDim");
        rightDim = hwMap.deviceInterfaceModule.get("rightDim");
        floor_seeker = hwMap.colorSensor.get("floor_seeker");
        frontColor = hwMap.colorSensor.get("frontColor");
        frontColor.enableLed(false);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        left_drive1 = hwMap.dcMotor.get("left_drive1");
        left_drive2 = hwMap.dcMotor.get("left_drive2");
        right_drive1 = hwMap.dcMotor.get("right_drive1");
        right_drive2 = hwMap.dcMotor.get("right_drive2");
        left_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive2.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        particle_collector = hwMap.dcMotor.get("particle_collector");
        particle_collector.setDirection(DcMotorSimple.Direction.REVERSE);
        mortar = hwMap.dcMotor.get("mortar");
        mortar.setDirection(DcMotorSimple.Direction.REVERSE);
        mortar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mortar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cap_ball_lift = hwMap.dcMotor.get("cap_ball_lift");
        cap_ball_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap_ball_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_beacon = hwMap.servo.get("left_beacon");
        right_beacon = hwMap.servo.get("right_beacon");
        right_beacon.setDirection(Servo.Direction.REVERSE);
        collector_gate = hwMap.servo.get("collector_gate");
        mortar_gate = hwMap.servo.get("mortar_gate");
        magazine_cam = hwMap.servo.get("magazine_cam");
        magazine_cam.setDirection(Servo.Direction.REVERSE);
        left_beacon.setPosition(leftBeaconIn);
        right_beacon.setPosition(rightBeaconIn);
        collector_gate.setPosition(PCGateDown);
        mortar_gate.setPosition(mortarGateDown);
        magazine_cam.setPosition(camDown);
    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

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

    public void stopMotors() {
        right_drive1.setPower(0);
        left_drive1.setPower(0);
        right_drive2.setPower(0);
        left_drive2.setPower(0);
    }

    public void setPowerLeft(double power) {
        left_drive1.setPower(power);
        left_drive2.setPower(power);
    }

    public void setPowerRight(double power) {
        right_drive1.setPower(power);
        right_drive2.setPower(power);
    }

    public void motorsRunUsingEncoder(){
        right_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void motorsStopAndReset(){
        right_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void resetEncoders(){
        motorsStopAndReset();
        motorsRunUsingEncoder();
    }
    public double getAdafruitHeading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            return -angles.firstAngle;

    }
}




