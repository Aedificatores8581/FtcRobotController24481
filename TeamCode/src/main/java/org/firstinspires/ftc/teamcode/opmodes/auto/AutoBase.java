package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LauncherConfig;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;

// shared steps for every auto
// extend this class and put the steps in runRoutine()
// flywheel spins up when the match starts
// everything stops when AUTO_PERIOD_SECONDS runs out, then onOverdrive() runs
// TODO: park from wherever we end up, using the odo pose
// TODO: skip shooting and go park when secondsLeft() is small
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
            robot.launcher.spinUp(LauncherConfig.TARGET_VEL); // spin up now, shoot later
            waitSeconds(RobotConstants.AUTO_START_DELAY_SECONDS); // alliance delay
            runRoutine();

            if (timeIsUp()) {
                overdriveRunning = true; // lets the steps in onOverdrive() run
                onOverdrive();
            }
        }

        robot.stop();
    }

    // steps

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

    // looks for a tag from minId to maxId
    // true as soon as one is seen
    // false if AUTO_TAG_SCAN_SECONDS runs out
    protected boolean waitForHiveTag(int minId, int maxId) {
        ElapsedTime scanTimer = new ElapsedTime();

        while (!shouldStop() && scanTimer.seconds() < RobotConstants.AUTO_TAG_SCAN_SECONDS) {
            boolean seen = robot.vision.isHiveTipped(minId, maxId);
            telemetry.addData("Scanning for tag", "%d-%d  seen: %s  %.1fs left", minId, maxId,
                    seen ? "YES" : "no", RobotConstants.AUTO_TAG_SCAN_SECONDS - scanTimer.seconds());
            tick(); // updates the Limelight too
            if (seen) {
                return true;
            }
        }
        return false;
    }

    // shoots count pieces
    // waits for the flywheel to be at speed before every shot
    // fires anyway after SHOOT_READY_TIMEOUT_SECONDS
    // timing values are in LauncherConfig
    protected void shoot(int count) {
        robot.launcher.spinUp(LauncherConfig.TARGET_VEL);

        for (int i = 0; i < count && !shouldStop(); i++) {
            waitForFlywheel(i + 1);
            telemetry.addData("Shots", "feeding %d of %d", i + 1, count);
            robot.launcher.feed();
            waitSeconds(LauncherConfig.SHOOT_FEED_HOLD_SECONDS);
            robot.launcher.retractFeeder();
            waitSeconds(LauncherConfig.SHOOT_FEED_PAUSE_SECONDS);
        }

        robot.launcher.stopFlywheel();
    }

    // drives out and looks for the hive tag
    // tag seen: back up, run the intake, shoot
    // no tag: just back up
    protected void shootAtHive(int minTagId, int maxTagId) {
        double p = RobotConstants.AUTO_POWER;

        driveInches(20, p);

        if (waitForHiveTag(minTagId, maxTagId)) {
            driveInches(19, -p);
            robot.intake.toggle();
            shoot(LauncherConfig.AUTO_SHOT_COUNT);
        } else {
            driveInches(8, -p);
        }
    }

    // helpers

    // waits until the flywheel has held its speed for READY_HOLD_SECONDS, or times out
    private void waitForFlywheel(int shotNumber) {
        ElapsedTime waitTimer = new ElapsedTime();
        ElapsedTime readyTimer = new ElapsedTime(); // how long it has been continuously ready
        boolean wasReady = false;
        boolean ready = false;

        while (!shouldStop() && waitTimer.seconds() < LauncherConfig.SHOOT_READY_TIMEOUT_SECONDS) {
            ready = robot.launcher.isReady();
            if (ready && !wasReady) {
                readyTimer.reset(); // just became ready, start counting
            }
            wasReady = ready;

            if (ready && readyTimer.seconds() >= LauncherConfig.READY_HOLD_SECONDS) {
                break;
            }
            telemetry.addData("Shoot", "Shot %d - waiting %.1fs", shotNumber, waitTimer.seconds());
            telemetry.addData("Flywheel error (ticks/s)", "%.0f (ready within %.0f)",
                    robot.launcher.getVelocity() - LauncherConfig.TARGET_VEL, LauncherConfig.VEL_TOLERANCE);
            tick();
        }

        boolean timedOut = !(ready && readyTimer.seconds() >= LauncherConfig.READY_HOLD_SECONDS);
        telemetry.addData("Shoot", "Shot %d - fired after %.1fs%s", shotNumber,
                waitTimer.seconds(), timedOut ? " (TIMED OUT)" : "");
    }

    private double distanceFrom(double startX, double startY) {
        return Math.hypot(robot.odo.getXInches() - startX, robot.odo.getYInches() - startY);
    }

    private boolean shouldStop() {
        return !opModeIsActive() || timeIsUp();
    }

    private boolean timeIsUp() {
        return !overdriveRunning && secondsLeft() <= 0;
    }

    /** One pass of the loop: show the match time and let every subsystem update. */
    private void tick() {
        telemetry.addData("Match time", "%.1f  (%.1f left)", matchTimer.seconds(), secondsLeft());
        robot.update();
    }

    private void stopAndPause() {
        robot.drive.drive(0, 0, 0);
        robot.update();
        sleep(RobotConstants.AUTO_PAUSE_MS);
    }
}
