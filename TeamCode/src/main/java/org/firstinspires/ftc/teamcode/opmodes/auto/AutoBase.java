package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.LauncherConfig;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;

/*
 * Shared setup and step helpers for autonomous routines.
 *
 * To write an auto: extend this class and fill in runRoutine() with steps like
 *     driveInches(24, RobotConstants.AUTO_POWER);
 *     turnToDegrees(90, RobotConstants.AUTO_POWER);
 *     shoot(4);
 *
 * Hive tipped check: robot.vision.isHiveTipped(minId, maxId), using one of the
 * RobotConstants.TAG_*_SCORING/AUDIENCE_MIN/MAX ranges for your alliance, then pick a plan.
 *
 * Safety: after AUTO_OVERDRIVE_SECONDS the remaining steps are skipped and
 * onOverdrive() runs instead (stop, or park).
 *
 * Alliance delay: set AUTO_START_DELAY_SECONDS in RobotConstants.
 * Time left: secondsLeft() tells a routine how much of the 30 seconds remains.
 *
 * TODO features to add:
 *  - Work out where we ended up (robot.odo pose) so we know how to get to park.
 *  - Park in the right zone: each auto gets a parkInZone() and overrides onOverdrive() to call it.
 *  - Use secondsLeft() to skip shooting and go park when time is short.
 */
public abstract class AutoBase extends LinearOpMode {

    protected Robot robot;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean overdriveRunning = false;
    private String lastShot = "none";

    // pose saved by markPose(), in odometry inches and degrees
    private double markX, markY, markHeading;

    // correction settings: ignore errors smaller than this, and move gently
    private static final double CORRECT_TOLERANCE_INCHES = 1.0;
    private static final double CORRECT_POWER = 0.25;
    private static final double CORRECT_MAX_SECONDS = 3.0; // the whole correction gives up after this

    // when set, every step loop stops at this match time (used to keep the correction from stalling)
    private double stepDeadline = Double.MAX_VALUE;

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
        robot.launcher.useAutoValues();

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
        log("driveInches " + inches + " at " + power);
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
        log("turnToDegrees " + targetDegrees + " at " + power);
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
        log("driveSeconds " + seconds + " at " + power);
        ElapsedTime timer = new ElapsedTime();
        while (!shouldStop() && timer.seconds() < seconds) {
            robot.drive.drive(power, 0, 0);
            tick();
        }
        stopAndPause();
    }

    // Strafe for a set time, for example to square up on a wall. Positive power is right.
    protected void strafeSeconds(double seconds, double power) {
        log("strafeSeconds " + seconds + " at " + power);
        ElapsedTime timer = new ElapsedTime();
        while (!shouldStop() && timer.seconds() < seconds) {
            robot.drive.drive(0, power, 0);
            tick();
        }
        stopAndPause();
    }

    // Gather balls along the wall. Start pointed at the gather direction (48 in from the wall).
    // Strafe into the wall to square up, drive 48 in with the intake on, strafe into the wall
    // again, then drive back to where we started.
    // wallPower: + if the wall is on the right, - if left.
    protected void gatherAlongWall(double wallPower) {
        double p = RobotConstants.AUTO_POWER;

        strafeSeconds(0.5, wallPower);   // square up on the wall
        robot.intake.on();
        driveInches(48, p); 
        driveInches(2, -p);    
        driveInches(2, p);   // gather
        driveInches(39.5, -p);             // back to the start spot
    }

    // Remember where we are so checkAndCorrect() can compare against it later.
    protected void markPose() {
        robot.update();
        markX = robot.odo.getXInches();
        markY = robot.odo.getYInches();
        markHeading = robot.odo.getHeadingDegrees();
        log(String.format("markPose x %.1f y %.1f h %.1f", markX, markY, markHeading));
    }

    // One small check and correction: turn to the marked heading, then fix any forward or
    // sideways error left over. Does not loop, so it cannot get stuck.
    protected void checkAndCorrect() {
        // skip it if we are short on time, so leave and park still gets to run
        if (timeToPark()) {
            log("check skipped, time to park");
            return;
        }
        stepDeadline = matchTimer.seconds() + CORRECT_MAX_SECONDS;
        turnToDegrees(markHeading, CORRECT_POWER);
        robot.update();

        double dx = markX - robot.odo.getXInches();
        double dy = markY - robot.odo.getYInches();
        double h = Math.toRadians(robot.odo.getHeadingDegrees());
        double forward = dx * Math.cos(h) + dy * Math.sin(h);   // + means we need to go forward
        double left = -dx * Math.sin(h) + dy * Math.cos(h);     // + means we need to go left
        log(String.format("check: need forward %.1f in, left %.1f in", forward, left));

        if (Math.abs(forward) > CORRECT_TOLERANCE_INCHES) {
            driveInches(Math.abs(forward), Math.signum(forward) * CORRECT_POWER);
        }
        if (Math.abs(left) > CORRECT_TOLERANCE_INCHES) {
            strafeInches(Math.abs(left), left > 0 ? -CORRECT_POWER : CORRECT_POWER); // + power is right
        }

        boolean timedOut = matchTimer.seconds() >= stepDeadline;
        stepDeadline = Double.MAX_VALUE;
        robot.update();
        if (timedOut) log("correction timed out, moving on");
        log(String.format("corrected: now x %.1f y %.1f h %.1f (target x %.1f y %.1f h %.1f)",
                robot.odo.getXInches(), robot.odo.getYInches(), robot.odo.getHeadingDegrees(),
                markX, markY, markHeading));
    }

    // Strafe a set distance using odometry. Positive power is right.
    protected void strafeInches(double inches, double power) {
        log("strafeInches " + inches + " at " + power);
        robot.update();
        double startX = robot.odo.getXInches();
        double startY = robot.odo.getYInches();

        while (!shouldStop() && distanceFrom(startX, startY) < inches) {
            robot.drive.drive(0, power, 0);
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
     * Look for a tag between minId and maxId for up to LauncherConfig.SCAN_TIME seconds.
     * True means the hive is tipped.
     */
    protected boolean scanForTip(int minId, int maxId) {
        ElapsedTime timer = new ElapsedTime();
        while (!shouldStop() && timer.seconds() < LauncherConfig.SCAN_TIME) {
            if (robot.vision.isHiveTipped(minId, maxId)) return true;
            telemetry.addData("Scan", "%.1fs left", LauncherConfig.SCAN_TIME - timer.seconds());
            tick();
        }
        return false;
    }

    /**
     * Shoot `count` pieces. Turns the intake on, and before each piece waits for the flywheel
     * to get back to speed (or READY_TIMEOUT), then opens the gate so the intake pushes a ball in.
     * Stops early if it's time to park. Turns the flywheel and intake off at the end.
     */
    protected void shoot(int count) {
        log("shoot " + count);
        robot.launcher.spinUp(); // safe if already spinning
        robot.intake.on();

        for (int i = 0; i < count && !shouldStop() && !timeToPark(); i++) {
            ElapsedTime readyTimer = new ElapsedTime();
            while (!shouldStop() && !timeToPark() && !robot.launcher.isReady()
                    && readyTimer.seconds() < LauncherConfig.READY_TIMEOUT) {
                tick();
            }
            if (timeToPark()) break;
            lastShot = String.format("Shot %d: %s at %.0f, waited %.1fs", i + 1,
                    robot.launcher.isReady() ? "READY" : "TIMEOUT",
                    robot.launcher.getVelocity(), readyTimer.seconds());
            RobotLog.ii("Auto", "%s (target %.0f, tolerance %.0f)", lastShot,
                    LauncherConfig.AUTO_TARGET_VELOCITY, LauncherConfig.VELOCITY_TOLERANCE);

            robot.launcher.feed();
            waitSeconds(LauncherConfig.FEED_HOLD_TIME);
            robot.launcher.retractFeeder();
            waitSeconds(LauncherConfig.FEED_PAUSE_TIME);
        }

        robot.launcher.stopFlywheel();
        robot.intake.off();
    }

    // ---- Helpers ----

    // Show the step on the driver station and write it to the robot log (Logcat tag: Auto)
    protected void log(String step) {
        RobotLog.ii("Auto", "%.1fs left: %s", secondsLeft(), step);
        telemetry.addData("Step", step);
        telemetry.update();
    }

    private double distanceFrom(double startX, double startY) {
        return Math.hypot(robot.odo.getXInches() - startX, robot.odo.getYInches() - startY);
    }

    private boolean timeToPark() {
        return secondsLeft() <= LauncherConfig.PARK_TIME;
    }

    private boolean shouldStop() {
        return !opModeIsActive() || timeIsUp() || matchTimer.seconds() >= stepDeadline;
    }

    private boolean timeIsUp() {
        return !overdriveRunning && matchTimer.seconds() >= RobotConstants.AUTO_OVERDRIVE_SECONDS;
    }

    /** One pass of the loop: show the match time and let every subsystem update. */
    private void tick() {
        telemetry.addData("Match time", "%.1f (%.1f left)", matchTimer.seconds(), secondsLeft());
        telemetry.addData("Last shot", lastShot);
        robot.update();
    }

    private void stopAndPause() {
        robot.drive.drive(0, 0, 0);
        robot.update();
        sleep(RobotConstants.AUTO_PAUSE_MS);
    }
}
