package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name = "goBILDA Pinpoint Example", group = "Linear OpMode")
//@Disabled
public class SensorGoBildaPinpointExample extends LinearOpMode {

    GoBildaPinpointDriver_v1 pinpoint;
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver_v1.class, "pinpoint");

        configurePinpoint();

        telemetry.addData("Status", "Initialized & Calibrated");
        telemetry.addData("Device Status", pinpoint.getDeviceStatus());
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            executeMotionAndRetraceRoutine();
        }

        while (opModeIsActive()) {
            pinpoint.update();
            displayTelemetry("Routine Finished");
            stopMotors();
        }
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver_v1.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set REVERSED on forward pod so driving FORWARD tracks as POSITIVE distance
        pinpoint.setEncoderDirections(GoBildaPinpointDriver_v1.EncoderDirection.REVERSED,
                GoBildaPinpointDriver_v1.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
        sleep(300);
    }

    private double getForwardDistance() {
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
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
        telemetry.addData("Device Status", pinpoint.getDeviceStatus());
        telemetry.addData("X (IN)", String.format(Locale.US, "%.2f", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Y (IN)", String.format(Locale.US, "%.2f", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Heading (DEG)", String.format(Locale.US, "%.2f", pose.getHeading(AngleUnit.DEGREES)));
        telemetry.update();
    }

    private void executeMotionAndRetraceRoutine() {
        // Step 1: Drive Forward 12 Inches
        while (opModeIsActive() && getForwardDistance() < 12.0) {
            pinpoint.update();
            setDrivePowers(0.3, 0.3, 0.3, 0.3);
            displayTelemetry("1. Driving Forward 12 IN");
        }
        stopMotors();
        sleep(500);

        // Step 2: Turn Left 90 Degrees
        while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < 90.0) {
            pinpoint.update();
            setDrivePowers(-0.3, -0.3, 0.3, 0.3); // Spin left
            displayTelemetry("2. Turning Left 90 DEG");
        }
        stopMotors();
        sleep(500);

        // Step 3: Drive Forward 12 Inches again
        double startDistStep3 = getForwardDistance();
        while (opModeIsActive() && (getForwardDistance() - startDistStep3) < 12.0) {
            pinpoint.update();
            setDrivePowers(0.3, 0.3, 0.3, 0.3);
            displayTelemetry("3. Driving Forward 12 IN Again");
        }
        stopMotors();
        sleep(1000);

        // --- RETRACE STEPS ---

        // Step 4a: Retrace - Drive Backwards 12 Inches
        while (opModeIsActive() && (getForwardDistance() - startDistStep3) > 0.0) {
            pinpoint.update();
            setDrivePowers(-0.3, -0.3, -0.3, -0.3);
            displayTelemetry("4a. Retracing: Backwards 12 IN");
        }
        stopMotors();
        sleep(500);

        // Step 4b: Retrace - Turn Right 90 Degrees back to 0° Heading
        while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) > 0.0) {
            pinpoint.update();
            setDrivePowers(0.3, 0.3, -0.3, -0.3); // Spin right
            displayTelemetry("4b. Retracing: Turning Right 90 DEG");
        }
        stopMotors();
        sleep(500);

        // Step 4c: Retrace - Drive Backwards 12 Inches to Origin (0,0)
        while (opModeIsActive() && getForwardDistance() > 0.0) {
            pinpoint.update();
            setDrivePowers(-0.3, -0.3, -0.3, -0.3);
            displayTelemetry("4c. Retracing: Backwards to Origin");
        }
        stopMotors();
    }
}
