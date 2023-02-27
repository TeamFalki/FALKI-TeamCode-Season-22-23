package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="TeleOpFALKI_V06", group="Linear Opmode")
public class FALKI_V06 extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor elevator = null;
    private Servo ga1 = null;
    private Servo ga2 = null;
    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rechtsVorne");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "linksVorne");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rechtsHinten");
        leftBackDrive = hardwareMap.get(DcMotor.class, "linksHinten");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        ga1 = hardwareMap.get(Servo.class, "ga1");
        ga2 = hardwareMap.get(Servo.class, "ga2");

        /** ##################################################################################
         ######**/
        // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
        //
/**##################################################################################
 ######**/
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same
//direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to
//ensure
        // that your motors are turning in the correct direction. So, start out with the reversals here,
//BUT
        // when you first test your robot, push the left joystick forward and observe the direction the
//wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick
//forward.

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        ga1.setDirection(Servo.Direction.FORWARD);
        ga2.setDirection(Servo.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {

            tgtPower = -this.gamepad2.left_stick_y;


            elevator.setPower(tgtPower);




            if (gamepad2.x) {

                ga1.setPosition(0.5);
                ga2.setPosition(0.5);
            }
            if (gamepad2.b) {

                ga1.setPosition(-0.5);
                ga2.setPosition(-0.5);
            }




            double max;
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
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            // 1) First get all the motors to take to correct positions on the robot
            // by adjusting your Robot Configuration if necessary.
            // 2) Then make sure they run in the correct direction by modifying the
            // the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.
 /*
 leftFrontPower = gamepad1.x ? 1.0 : 0.0; // X gamepad
 leftBackPower = gamepad1.a ? 1.0 : 0.0; // A gamepad
 rightFrontPower = gamepad1.y ? 1.0 : 0.0; // Y gamepad
 rightBackPower = gamepad1.b ? 1.0 : 0.0; // B gamepad
 */
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower*(gamepad1.right_bumper ? 1 : 0.5));
            rightFrontDrive.setPower(rightFrontPower*(gamepad1.right_bumper ? 1 : 0.5));
            leftBackDrive.setPower(leftBackPower*(gamepad1.right_bumper ? 1 : 0.5));
            rightBackDrive.setPower(rightBackPower*(gamepad1.right_bumper ? 1 : 0.5));
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Status", "Running");
            telemetry.addData("Servo Position", ga1.getPosition());
            telemetry.addData("Servo Position", ga2.getPosition());
            telemetry.addData("Motor Power", elevator.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }}