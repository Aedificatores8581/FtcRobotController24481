
package org.firstinspires.ftc.teamcode;

// Import required FTC SDK packages for Pinpoint driver, OpMode, and motors
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

// Declare this class as an Autonomous OpMode so it appears on the Driver Station screen
@Autonomous(name = "goBILDA® Pinpoint Odometry Computer v2", group = "Sensor")
//@Disabled // Uncomment @Disabled if you want to hide this OpMode from the Driver Station list
public class GoBildaPinpointDriver_v2 extends LinearOpMode {

    // Declare hardware objects
    GoBildaPinpointDriver pinpoint; // Reference to the goBILDA Pinpoint Odometry Computer module
    private DcMotor frontLeftDrive = null;  // Front Left drivetrain motor
    private DcMotor backLeftDrive = null;   // Back Left drivetrain motor
    private DcMotor frontRightDrive = null; // Front Right drivetrain motor
    private DcMotor backRightDrive = null;  // Back Right drivetrain motor

    /**
     * This is the main function that runs when you select the OpMode on the Driver Station.
     */
    @Override
    public void runOpMode() {
        // Step 1: Map motor variables to the exact motor names configured on the Robot Controller app
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "FL");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "BL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backRightDrive  = hardwareMap.get(DcMotor.class, "BR");

        // Step 2: Reverse the left side motors so positive motor power drives the robot forward
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Step 3: Get reference to the Pinpoint computer hardware object named "pinpoint"
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Step 4: Configure pod offsets, pod resolution (4-Bar Pods), and reset position
        configureAndRecalibratePinpoint();

        // Step 5: Display initialization telemetry on Driver Station screen
        telemetry.addData("Status", "Initialized & Calibrated");
        telemetry.addData("Device Status", pinpoint.getDeviceStatus());
        telemetry.addData("Device Connection Info", pinpoint.getDeviceClient().getConnectionInfo());
        telemetry.addData("Device Version", pinpoint.getDeviceVersion());
        telemetry.addData("Frequency (Hz)", String.format(Locale.US, "%.1f", pinpoint.getFrequency()));
        telemetry.update();

        // Step 6: Wait here until the driver presses the START button on the Driver Station
        waitForStart();

        // Step 7: Once START is pressed, execute the autonomous movements routine
        if (opModeIsActive()) {
            executeMotionAndRetraceRoutine();
        }

        // Step 8: After routine finishes, enter idle loop to keep displaying telemetry until match ends
        while (opModeIsActive()) {
            pinpoint.update(); // Continually update Pinpoint position readings
            displayTelemetry("Routine Finished"); // Show final X, Y, Heading on screen
            stopMotors(); // Keep motors powered off
        }
    }

    /**
     * Configures Pinpoint pod offsets, encoder resolution, encoder directions, and recalibrates IMU.
     * NOTE: The robot MUST remain completely stationary while this function runs!
     */
    public void configureAndRecalibratePinpoint() {
        // Set pod offsets in millimeters from center of robot rotation (X = -84mm, Y = -168mm)
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.INCH);

        // Set resolution for the goBILDA 4-Bar Pods (~19.894 ticks per mm)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set direction of encoders: Forward pod REVERSED so moving forward = positive inches
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset position to (0,0,0) and recalibrate IMU (robot MUST be still!)
        telemetry.addData("Status", "Resetting Position and IMU...");
        telemetry.update();
        pinpoint.resetPosAndIMU();
        sleep(500); // Allow 0.5 seconds for hardware reset to complete

        // Execute bulk read to populate device status & initial position
        pinpoint.update();
    }

    /**
     * Helper method to get current forward position in Inches from Pinpoint.
     * @return current Y position in inches
     */
    private double getForwardDistanceInches() {
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
    }

    /**
     * Helper method to set motor power to all 4 drivetrain motors simultaneously.
     * @param fl Front Left power (-1.0 to 1.0)
     * @param bl Back Left power (-1.0 to 1.0)
     * @param fr Front Right power (-1.0 to 1.0)
     * @param br Back Right power (-1.0 to 1.0)
     */
    private void setDrivePowers(double fl, double bl, double fr, double br) {
        frontLeftDrive.setPower(fl);
        backLeftDrive.setPower(bl);
        frontRightDrive.setPower(fr);
        backRightDrive.setPower(br);
    }

    /**
     * Helper method to immediately stop all 4 drive motors.
     */
    private void stopMotors() {
        setDrivePowers(0, 0, 0, 0);
    }

    /**
     * Helper method to update telemetry on the Driver Station screen.
     * @param currentAction Description of what the robot is currently doing
     */
    private void displayTelemetry(String currentAction) {
        Pose2D pose = pinpoint.getPosition(); // Read latest pose object from Pinpoint
        telemetry.addData("Action", currentAction);
        telemetry.addData("Device Status", pinpoint.getDeviceStatus());
        telemetry.addData("Connection Info", pinpoint.getDeviceClient().getConnectionInfo());
        telemetry.addData("Encoder X Ticks", pinpoint.getEncoderX());
        telemetry.addData("Encoder Y Ticks", pinpoint.getEncoderY());
        telemetry.addData("X (IN)", String.format(Locale.US, "%.2f", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Y (IN)", String.format(Locale.US, "%.2f", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Heading (DEG)", String.format(Locale.US, "%.2f", pose.getHeading(AngleUnit.DEGREES)));
        telemetry.update();
    }

    /**
     * MAIN AUTONOMOUS ROUTINE:
     * This method executes a full test routine:
     *   1. Drives Forward 12 Inches
     *   2. Turns Left 90 Degrees
     *   3. Drives Forward 12 Inches
     *   4. Retraces steps backwards to starting position (0,0)
     * YOU CAN EDIT OR REPLACE ANYTHING IN THIS FUNCTION USING THE COPY-PASTE CHEAT SHEET AT THE TOP!
     */
    private void executeMotionAndRetraceRoutine() {
        // ------------------------------------------------------------------------------------------
        // STEP 1: DRIVE FORWARD 12 INCHES
        // ------------------------------------------------------------------------------------------

        pinpoint.update(); // Fetch current position before starting step
        double startingpoint = getForwardDistanceInches();
        while (opModeIsActive() && (getForwardDistanceInches() - startingpoint) < 12.0) {
            setDrivePowers(-0.3, -0.3, -0.3, -0.3); // Drive forward at 30% speed
            displayTelemetry("1. Driving Forward 12 IN");
            pinpoint.update(); // Update Pinpoint position


        }
        stopMotors(); // Stop wheels
        sleep(500);   // Wait 0.5s to settle

        // ------------------------------------------------------------------------------------------
        // STEP 2: TURN LEFT 90 DEGREES
        // ------------------------------------------------------------------------------------------
        pinpoint.update();
        while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < 90.0) {
            setDrivePowers(-0.3, -0.3, 0.3, 0.3); // Spin left (Left wheels -, Right wheels +)
            displayTelemetry("2. Turning Left 90 DEG");
            pinpoint.update();
        }
        stopMotors();
        sleep(500);

        // ------------------------------------------------------------------------------------------
        // STEP 3: DRIVE FORWARD 12 INCHES AGAIN (RELATIVE DISTANCE)
        // ------------------------------------------------------------------------------------------
        pinpoint.update();
        double startDistStep3 = getForwardDistanceInches(); // Record start position of Step 3
        while (opModeIsActive() && (getForwardDistanceInches() - startDistStep3) < -12.0) {
            setDrivePowers(-0.3, -0.3, -0.3, -0.3);
            displayTelemetry("3. Driving Forward 12 IN Again");
            pinpoint.update();
        }
        stopMotors();
        sleep(1000);

        // ------------------------------------------------------------------------------------------
        // STEP 4A: RETRACE - DRIVE BACKWARDS 12 INCHES
        // ------------------------------------------------------------------------------------------
        pinpoint.update();
        while (opModeIsActive() && (getForwardDistanceInches() - startDistStep3) > 0.0) {
            setDrivePowers(0.3, 0.3, 0.3, 0.3); // Drive backwards
            displayTelemetry("4a. Retracing: Backwards 12 IN");
            pinpoint.update();
        }
        stopMotors();
        sleep(500);

        // ------------------------------------------------------------------------------------------
        // STEP 4B: RETRACE - TURN RIGHT 90 DEGREES BACK TO 0 DEGREE HEADING
        // ------------------------------------------------------------------------------------------
        pinpoint.update();
        while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) > 0.0) {
            setDrivePowers(0.3, 0.3, -0.3, -0.3); // Spin right (Left wheels +, Right wheels -)
            displayTelemetry("4b. Retracing: Turning Right 90 DEG");
            pinpoint.update();
        }
        stopMotors();
        sleep(500);

        // ------------------------------------------------------------------------------------------
        // STEP 4C: RETRACE - DRIVE BACKWARDS TO ORIGIN (0,0)
        // ------------------------------------------------------------------------------------------
        pinpoint.update();
        while (opModeIsActive() && getForwardDistanceInches() > 0.0) {
            setDrivePowers(0.3, 0.3, 0.3, 0.3);
            displayTelemetry("4c. Retracing: Backwards to Origin");
            pinpoint.update();
        }
        stopMotors(); // Final stop
    }
}
