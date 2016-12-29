package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class  Beacon extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;
    double left;
    double right;
    double turnTolerance = 1;
    double newHeading;
    double oneRotation =420;
    double currentHeading;
    double driveGain = .007;
    double directionAdjust;
    double startHeading;
    double headingError;
    double driveSteering;
    double leftPower;
    double rightPower;
    Servo armServo;
    DcMotor leftMotor;
    DcMotor rightMotor;
    //DcMotor shooterMotor;
    GyroSensor gyro;
    ModernRoboticsI2cRangeSensor Range;
    ModernRoboticsI2cRangeSensor rightRangeSensor;
    ColorSensor floorSeeker;
    ColorSensor beaconSeeker;
    String color;

    @Override
    public void init() {
        I2cAddr colorAdress = I2cAddr.create8bit(0x70);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        Range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        floorSeeker = hardwareMap.colorSensor.get("floorSeeker");
        rightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRangeSensor");
        armServo = hardwareMap.servo.get("armServo");
        armServo.setPosition(0);
        beaconSeeker = hardwareMap.colorSensor.get("beaconSeeker");
        beaconSeeker.enableLed(false);
        beaconSeeker.setI2cAddress(colorAdress);
        // Wait for the game to start (driver presses PLAY)
        gyro.calibrate();
        while (gyro.isCalibrating()) {
        }

    }
    @Override
    public void loop() {


        if(gamepad1.a){
            armServo.setPosition(.15);
        }else{
            armServo.setPosition(0);
        }
        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;
        right = com.qualcomm.robotcore.util.Range.clip(right, -1, 1);
        left = com.qualcomm.robotcore.util.Range.clip(left, -1, 1);
        leftMotor.setPower(left);
        rightMotor.setPower(right);
        telemetry.addData("red", beaconSeeker.red());
        telemetry.addData("blue", beaconSeeker.blue());
        if (beaconSeeker.red() > 1.5) {
            color = "red";
        } else if (beaconSeeker.blue() > 1.5) {
            color = "blue";
        }

        telemetry.addData("color", color);


    }

    }
