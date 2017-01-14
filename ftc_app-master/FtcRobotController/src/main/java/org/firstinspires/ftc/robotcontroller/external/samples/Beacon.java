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
    Servo right_beacon;
    Servo left_beacon;
    DcMotor left_drive1;
    DcMotor left_drive2;
    DcMotor right_drive1;
    DcMotor right_drive2;
    DcMotor mortar;
    GyroSensor gyro;
    ModernRoboticsI2cRangeSensor right_range;
    ModernRoboticsI2cRangeSensor left_range;
    ModernRoboticsI2cRangeSensor front_range;
    ColorSensor floor_seeker;
    ColorSensor right_color;
    ColorSensor left_color;
    String color;

    @Override
    public void init() {
        I2cAddr colorAddress = I2cAddr.create8bit(0x70);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        left_drive1 = hardwareMap.dcMotor.get("left_drive1");
        left_drive2 = hardwareMap.dcMotor.get("left_drive2");
        right_drive1 = hardwareMap.dcMotor.get("right_drive1");
        right_drive2 = hardwareMap.dcMotor.get("right_drive2");
        right_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        mortar = hardwareMap.dcMotor.get("mortar");
        gyro = hardwareMap.gyroSensor.get("gyro");
        right_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "right_range");
        left_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_range");
        floor_seeker = hardwareMap.colorSensor.get("floor_seeker");
        left_beacon = hardwareMap.servo.get("left_beacon");
        right_beacon = hardwareMap.servo.get("right_beacon");
        left_beacon.setPosition(0);
        right_beacon.setPosition(0);
        left_color = hardwareMap.colorSensor.get("left_color");
        right_color = hardwareMap.colorSensor.get("right_color");
        left_color.setI2cAddress(colorAddress);
        right_color.setI2cAddress(colorAddress);
        // Wait for the game to start (driver presses PLAY)
        gyro.calibrate();
        while (gyro.isCalibrating()) {
        }

    }
    @Override
    public void loop() {


        if(gamepad1.a){
            right_beacon.setPosition(.15);
        }else{
            right_beacon.setPosition(0);
        }
        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;
        right = com.qualcomm.robotcore.util.Range.clip(right, -1, 1);
        left = com.qualcomm.robotcore.util.Range.clip(left, -1, 1);


        //right = (float) scaleInput(right);
        //left = (float) scaleInput(left);
        //left_drive1.setPower(left);
        //left_drive2.setPower(left);
        //right_drive1.setPower(right);
        //right_drive2.setPower(right);

        telemetry.addData("red", right_color.red());
        telemetry.addData("blue", right_color.blue());
        if (right_color.red() > 1.5) {
            color = "red";

        } else if (right_color.blue() > 1.5) {
            color = "blue";
        }

        telemetry.addData("color", color);


    }

    }
