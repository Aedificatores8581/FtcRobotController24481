package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/*
 * Autonomous mode using the goBILDA Pinpoint for distance and heading.
 *
 * The routine is the list of calls after waitForStart(). Add, remove, or reorder
 * lines there to change what the robot does. Names, directions, and power levels
 * live in RobotConstants.java.
 *
 * Logging: RobotLog lines are tagged "SimpleAuto". To watch them live in a terminal:
 *     adb logcat -s SimpleAuto
 * or type SimpleAuto in the Logcat filter box in Android Studio. To save to a file:
 *     adb logcat -s SimpleAuto > ~/robot_log.txt
 * Levels: RobotLog.vv (verbose), dd (debug), ii (info), ww (warn), ee (error).
 *
 * Distances are in inches, angles in degrees. Negative = opposite direction:
 *     driveForward(-12)  drives backward 12 inches
 *     turnLeft(-90)      turns right 90 degrees
 */
@Autonomous(name = "Simple Auto", group = "Auto")
public class SimpleAuto extends LinearOpMode {

    private static final String TAG = "SimpleAuto";

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        // ---- INIT: runs when INIT is pressed. Robot must be still (Pinpoint calibrates here). ----
        RobotLog.ii(TAG, "INIT: setting up hardware");
        initHardware();
        RobotLog.ii(TAG, "INIT done. Pinpoint status: %s", pinpoint.getDeviceStatus());

        telemetry.addData("Status", "Ready. Press START.");
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.update();

        waitForStart();
        RobotLog.ii(TAG, "START pressed, running routine");

        // ---- ROUTINE: runs once, top to bottom, after START. Edit this list. ----
        driveForward(12);
        // turnLeft(90);
        // strafeLeft(6);
        // driveForward(-12);

        // ---- DONE ----
        RobotLog.ii(TAG, "Routine finished");
        telemetry.addData("Status", "Done");
        telemetry.update();
        while (opModeIsActive()) {
            idle(); // keep the final telemetry on screen until STOP or the 30s timer
        }
    }

    // ------------------------------------------------------------------ setup

    private void initHardware() {
        frontLeft  = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_LEFT_MOTOR);
        backLeft   = hardwareMap.get(DcMotor.class, RobotConstants.BACK_LEFT_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_RIGHT_MOTOR);
        backRight  = hardwareMap.get(DcMotor.class, RobotConstants.BACK_RIGHT_MOTOR);

        frontLeft.setDirection(RobotConstants.FRONT_LEFT_DIRECTION);
        backLeft.setDirection(RobotConstants.BACK_LEFT_DIRECTION);
        frontRight.setDirection(RobotConstants.FRONT_RIGHT_DIRECTION);
        backRight.setDirection(RobotConstants.BACK_RIGHT_DIRECTION);

        // Brake at zero power instead of coasting so the robot stops where we tell it to.
        for (DcMotor m : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.PINPOINT);
        pinpoint.setOffsets(RobotConstants.X_POD_OFFSET_MM, RobotConstants.Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(RobotConstants.POD_TYPE);
        pinpoint.setEncoderDirections(RobotConstants.X_POD_DIRECTION, RobotConstants.Y_POD_DIRECTION);
        pinpoint.resetPosAndIMU(); // zeros position and recalibrates the gyro; robot must be still
    }

    // ------------------------------------------------------------------ moves

    /** Drive straight forward (negative = backward). */
    private void driveForward(double inches) {
        double p = Math.signum(inches) * RobotConstants.DRIVE_POWER;
        RobotLog.ii(TAG, "driveForward(%.1f) power=%.2f", inches, p);
        setPowers(p, p, p, p);
        waitForDistance(Math.abs(inches), "Forward " + inches + " in");
        stopMotors();
    }

    /** Strafe sideways to the left (negative = right). */
    private void strafeLeft(double inches) {
        double p = Math.signum(inches) * RobotConstants.STRAFE_POWER;
        // Mecanum strafe left: front-left & back-right roll backward, the other two roll forward.
        RobotLog.ii(TAG, "strafeLeft(%.1f) power=%.2f", inches, p);
        setPowers(-p, p, p, -p);
        waitForDistance(Math.abs(inches), "Strafe " + inches + " in");
        stopMotors();
    }

    /** Turn in place to the left / counter-clockwise (negative = right). */
    private void turnLeft(double degrees) {
        double p = Math.signum(degrees) * RobotConstants.TURN_POWER;
        // Left side backward, right side forward = counter-clockwise.
        RobotLog.ii(TAG, "turnLeft(%.1f) power=%.2f", degrees, p);
        setPowers(-p, -p, p, p);

        pinpoint.update();
        // Unnormalized heading keeps counting past 180, so crossing it mid-turn is not a problem.
        double startHeading = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
        while (opModeIsActive()) {
            pinpoint.update();
            double turned = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES) - startHeading;
            if (Math.abs(turned) >= Math.abs(degrees)) {
                RobotLog.ii(TAG, "turnLeft done: turned %.1f of %.1f deg", turned, Math.abs(degrees));
                break;
            }
            report("Turn " + degrees + " deg", Math.abs(degrees), Math.abs(turned));
        }
        stopMotors();
    }

    // ---------------------------------------------------------------- helpers

    private void setPowers(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    private void stopMotors() {
        setPowers(0, 0, 0, 0);
    }

    /**
     * Block until the robot has moved `inches` from where it is right now.
     * Measures straight-line distance from the start point, so it works no matter which way
     * the robot is facing (Pinpoint X/Y are field-relative, not robot-relative).
     */
    private void waitForDistance(double inches, String label) {
        pinpoint.update();
        double startX = pinpoint.getPosX(DistanceUnit.INCH);
        double startY = pinpoint.getPosY(DistanceUnit.INCH);

        while (opModeIsActive()) {
            pinpoint.update(); // one I2C read; the getters below return values cached by this call
            double dx = pinpoint.getPosX(DistanceUnit.INCH) - startX;
            double dy = pinpoint.getPosY(DistanceUnit.INCH) - startY;
            double traveled = Math.hypot(dx, dy);
            if (traveled >= inches) {
                // One line per move, not per loop pass: logging every iteration floods Logcat.
                RobotLog.ii(TAG, "%s done: traveled %.2f of %.1f in", label, traveled, inches);
                break;
            }
            report(label, inches, traveled);
        }
    }

    private void report(String move, double target, double current) {
        telemetry.addData("Move", move);
        telemetry.addData("Progress", "%.1f / %.1f", current, target);
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.update();
    }
}
