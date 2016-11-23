

        package org.firstinspires.ftc.robotcontroller.external.samples;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.GyroSensor;
        import com.qualcomm.robotcore.hardware.Servo;


        public class Breadboard extends OpMode{
            double left;
            double right;
            DcMotor leftDriveMotor1;
            DcMotor leftDriveMotor2;
            Servo rightServo;
            Servo leftServo;

            /*
             * Code to run ONCE when the driver hits INIT
             */
            @Override
            public void init() {

            /* Initialize the hardware variables.
             * The init() method of the hardware class does all the work here
             */



                leftDriveMotor1=hardwareMap.dcMotor.get("left_drive");
                leftDriveMotor2=hardwareMap.dcMotor.get("right_drive");

                leftServo=hardwareMap.servo.get("left_servo");
                rightServo=hardwareMap.servo.get("right_servo");
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
                // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
                left = -gamepad1.left_stick_y;
                right = -gamepad1.right_stick_y;
                leftDriveMotor1.setPower(left);
                leftDriveMotor2.setPower(right);
                left = -gamepad2.left_stick_y;
                right = -gamepad2.right_stick_y;
                leftServo.setPosition(0.75);
                rightServo.setPosition(-0.75);
            }

            /*
             * Code to run ONCE after the driver hits STOP
             */
            @Override
            public void stop() {
            }

        }

