package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpTeamFalki", group = "Linear Opmode")
public class TeleOpTeamFalki extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();
    double multiplication;
    double elevatorPower;
    double max;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rechtsVorne");
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "linksVorne");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rechtsHinten");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "linksHinten");

        DcMotor elevator = hardwareMap.get(DcMotor.class, "elevator");
        Servo greifArm1 = hardwareMap.get(Servo.class, "greifArm1");
        Servo greifArm2 = hardwareMap.get(Servo.class, "greifArm2");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        elevator.setDirection(DcMotor.Direction.FORWARD);
        greifArm1.setDirection(Servo.Direction.FORWARD);
        greifArm2.setDirection(Servo.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double elevatorc1 = 0;
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            elevatorPower = -gamepad2.left_stick_y - elevatorc1;
            elevator.setPower(elevatorPower);


                if (gamepad1.dpad_down) {
                                elevatorc1 = 0.5;
                            } else {
                if (gamepad1.dpad_up) {
                                elevatorc1 = -0.5;
                            } else {
                                elevatorc1 = 0;
                            }
            }

            if (gamepad2.x) { //close
                greifArm1.setPosition(0.5);
                greifArm2.setPosition(0.5);
            }
            if (gamepad2.b) { //open
                greifArm1.setPosition(-0.5);
                greifArm2.setPosition(-0.5);
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (gamepad1.right_bumper) {
                multiplication = 1;
            } else if (gamepad1.left_bumper) {
                multiplication = 0.25;
            } else {
                multiplication = 0.5;
            }

            if (gamepad1.x) { //close
                greifArm1.setPosition(0.5);
                greifArm2.setPosition(0.5);
            }
            if (gamepad1.b) { //open
                greifArm1.setPosition(-0.5);
                greifArm2.setPosition(-0.5);
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * multiplication);
            rightFrontDrive.setPower(rightFrontPower * multiplication);
            leftBackDrive.setPower(leftBackPower * multiplication);
            rightBackDrive.setPower(rightBackPower * multiplication);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Servo Position", greifArm1.getPosition());
            telemetry.addData("Servo Position", greifArm2.getPosition());
            telemetry.addData("Motor Power", elevator.getPower());
            telemetry.addData("Elevator Power", elevatorPower);
            telemetry.addData("Status", "Running");
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }
    }
}
