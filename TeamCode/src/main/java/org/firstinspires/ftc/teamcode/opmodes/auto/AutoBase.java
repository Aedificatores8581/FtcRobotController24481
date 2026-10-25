package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;

/*
 * Shared setup and step helpers for autonomous routines.
 *
 * To write an auto: extend this class and fill in runRoutine() with steps like
 *     driveInches(24, RobotConstants.AUTO_POWER);
 *     turnToDegrees(90, RobotConstants.AUTO_POWER);
 *
 * Safety: after AUTO_OVERDRIVE_SECONDS the remaining steps are skipped and
 * onOverdrive() runs instead (stop, or park).
 *
 * Alliance delay: set AUTO_START_DELAY_SECONDS in RobotConstants.
 * Time left: secondsLeft() tells a routine how much of the 30 seconds remains.
 *
 * TODO features to add:
 *  - Hive tipped check: Vision.isHiveTipped() (AprilTag), then pick the plan.
 *  - shoot(count): shoot the loaded pieces (stub below).
 *  - Work out where we ended up (robot.odo pose) so we know how to get to park.
 *  - Park in the right zone: each auto gets a parkInZone() and overrides onOverdrive() to call it.
 *  - Use secondsLeft() to skip shooting and go park when time is short.
 */
public abstract class AutoBase extends LinearOpMode {

    protected Robot robot;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean overdriveRunning = false;

    /** Each auto puts its steps here. */
    protected abstract void runRoutine();

    /** Runs if time ran out before the routine finished. TODO: park. */
    protected void onOverdrive() {
        robot.drive.drive(0, 0, 0);
    }

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();

        waitForStart();
        matchTimer.reset();

        if (opModeIsActive()) {
            waitSeconds(RobotConstants.AUTO_START_DELAY_SECONDS); // alliance delay
            runRoutine();

            if (timeIsUp()) {
                overdriveRunning = true; // lets the steps in onOverdrive() run
                onOverdrive();
            }
        }

        robot.stop();
    }

    // ---- Steps ----

    /** Drive in a straight line. Positive power is forward, negative is backward. */
    protected void driveInches(double inches, double power) {
        robot.update();
        double startX = robot.odo.getXInches();
        double startY = robot.odo.getYInches();

        while (!shouldStop() && distanceFrom(startX, startY) < inches) {
            robot.drive.drive(power, 0, 0);
            tick();
        }
        stopAndPause();
    }

    /** Turn until the heading passes the target. 0 is straight ahead, positive is left. */
    protected void turnToDegrees(double targetDegrees, double power) {
        robot.update();
        boolean turnLeft = targetDegrees > robot.odo.getHeadingDegrees();

        while (!shouldStop() && (turnLeft
                ? robot.odo.getHeadingDegrees() < targetDegrees
                : robot.odo.getHeadingDegrees() > targetDegrees)) {
            robot.drive.drive(0, 0, turnLeft ? -power : power); // positive turn is clockwise
            tick();
        }
        stopAndPause();
    }

    /** Drive for a set time, for example to push into a wall. */
    protected void driveSeconds(double seconds, double power) {
        ElapsedTime timer = new ElapsedTime();
        while (!shouldStop() && timer.seconds() < seconds) {
            robot.drive.drive(power, 0, 0);
            tick();
        }
        stopAndPause();
    }

    /** Wait while the robot keeps updating (telemetry, odometry). */
    protected void waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (!shouldStop() && timer.seconds() < seconds) {
            tick();
        }
    }

    /** Seconds left in the autonomous period. */
    protected double secondsLeft() {
        return RobotConstants.AUTO_PERIOD_SECONDS - matchTimer.seconds();
    }

    /**
     * Shoot the loaded pieces. Not built yet.
     * TODO:
     *  1. robot.launcher.spinUp(LauncherConfig.DEFAULT_RPM)
     *  2. wait until robot.launcher.isReady() (give up after a few seconds)
     *  3. robot.launcher.feed(), short wait, robot.launcher.retractFeeder()
     *  4. repeat for each piece, then robot.launcher.stopFlywheel()
     */
    protected void shoot(int count) {
        // TODO
    }

    // ---- Helpers ----

    private double distanceFrom(double startX, double startY) {
        return Math.hypot(robot.odo.getXInches() - startX, robot.odo.getYInches() - startY);
    }

    private boolean shouldStop() {
        return !opModeIsActive() || timeIsUp();
    }

    private boolean timeIsUp() {
        return !overdriveRunning && matchTimer.seconds() >= RobotConstants.AUTO_OVERDRIVE_SECONDS;
    }

    /** One pass of the loop: show the match time and let every subsystem update. */
    private void tick() {
        telemetry.addData("Match time", "%.1f", matchTimer.seconds());
        robot.update();
    }

    private void stopAndPause() {
        robot.drive.drive(0, 0, 0);
        robot.update();
        sleep(RobotConstants.AUTO_PAUSE_MS);
    }
}
