package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name = "goBILDA® Pinpoint Odometry Computer v2", group = "Sensor")
public class GoBildaPinpointDriver_v2 extends LinearOpMode {

    // --- Hardware Declarations ---
    GoBildaPinpointDriver pinpoint;
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    // --- Main OpMode Lifecycle ---
    @Override
    public void runOpMode() {
        // Stream telemetry to both Driver Station and FTC Dashboard (192.168.43.1:8080/dash)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // --- Drivetrain Initialization ---
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "FL");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "BL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backRightDrive  = hardwareMap.get(DcMotor.class, "BR");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // --- Pinpoint Initialization & Calibration ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configureAndRecalibratePinpoint();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Device Status", pinpoint.getDeviceStatus());
        telemetry.update();

        waitForStart();

        // --- Autonomous Execution ---
        if (opModeIsActive()) {
            executeMotionAndRetraceRoutine();
        }

        // --- Idle Completion Loop ---
        while (opModeIsActive()) {
            pinpoint.update();
            displayTelemetry("Finished");
            stopMotors();
        }
    }

    // --- Pinpoint Configuration ---
    public void configureAndRecalibratePinpoint() {
        // Pod offsets converted from MM (-84mm, -168mm) to INCHES (-3.307 in, -6.614 in)
        pinpoint.setOffsets(-3.307, -6.614, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
        sleep(500);
        pinpoint.update();
    }

    // --- Helper Methods ---
    private double getDistanceInches(Pose2D start, Pose2D current) {
        double dx = current.getX(DistanceUnit.INCH) - start.getX(DistanceUnit.INCH);
        double dy = current.getY(DistanceUnit.INCH) - start.getY(DistanceUnit.INCH);
        return Math.hypot(dx, dy);
    }

    private void setDrivePowers(double fl, double bl, double fr, double br) {
        frontLeftDrive.setPower(fl);
        backLeftDrive.setPower(bl);
        frontRightDrive.setPower(fr);
        backRightDrive.setPower(br);
    }

    private void stopMotors() {
        setDrivePowers(0, 0, 0, 0);
    }

    private void displayTelemetry(String currentAction) {
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Action", currentAction);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Forward X (IN)", String.format(Locale.US, "%.2f", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Strafe Y (IN)", String.format(Locale.US, "%.2f", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Heading (DEG)", String.format(Locale.US, "%.2f", pose.getHeading(AngleUnit.DEGREES)));
        telemetry.update();
    }

    // --- Autonomous Motion Routine ---
    private void executeMotionAndRetraceRoutine() {
        // Section 1: Drive Forward 12 Inches
        pinpoint.update();
        Pose2D startPose1 = pinpoint.getPosition();
        while (opModeIsActive() && getDistanceInches(startPose1, pinpoint.getPosition()) < 12.0) {
            setDrivePowers(-0.3, -0.3, -0.3, -0.3);
            displayTelemetry("1. Forward 12 IN");
            pinpoint.update();
        }
        stopMotors();
        sleep(500);

        // Section 2: Turn Left 90 Degrees
        pinpoint.update();
        while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < 90.0) {
            setDrivePowers(-0.3, -0.3, 0.3, 0.3);
            displayTelemetry("2. Turn Left 90 DEG");
            pinpoint.update();
        }
        stopMotors();
        sleep(500);

        // Section 3: Drive Forward 12 Inches Again
        pinpoint.update();
        Pose2D startPose3 = pinpoint.getPosition();
        while (opModeIsActive() && getDistanceInches(startPose3, pinpoint.getPosition()) < 12.0) {
            setDrivePowers(-0.3, -0.3, -0.3, -0.3);
            displayTelemetry("3. Forward 12 IN Again");
            pinpoint.update();
        }
        stopMotors();
        
        // --- TRANSITION FROM SECTION 3 TO 4A ---
        pinpoint.update();
        displayTelemetry("Section 3 Complete (12 IN) -> Retracing Home");
        sleep(1000);

        // Section 4a: Retrace Backwards 12 Inches
        pinpoint.update();
        Pose2D startPose4a = pinpoint.getPosition();
        while (opModeIsActive() && getDistanceInches(startPose4a, pinpoint.getPosition()) < 12.0) {
            setDrivePowers(0.3, 0.3, 0.3, 0.3);
            displayTelemetry("4a. Backwards 12 IN");
            pinpoint.update();
        }
        stopMotors();
        sleep(500);

        // Section 4b: Retrace Turn Right 90 Degrees back to exact Heading 0.0°
        pinpoint.update();
        while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) > 0.0) {
            setDrivePowers(0.3, 0.3, -0.3, -0.3);
            displayTelemetry("4b. Turn Right to 0.0 DEG");
            pinpoint.update();
        }
        stopMotors();
        sleep(500);

        // Section 4c: Retrace Backwards to exact Origin (X = 0.0 IN)
        pinpoint.update();
        while (opModeIsActive() && pinpoint.getPosition().getX(DistanceUnit.INCH) > 0.0) {
            setDrivePowers(0.3, 0.3, 0.3, 0.3);
            displayTelemetry("4c. Backwards to Origin (0.0 IN)");
            pinpoint.update();
        }
        stopMotors();
        displayTelemetry("5. STOP (0.0 IN)");
    }
}
