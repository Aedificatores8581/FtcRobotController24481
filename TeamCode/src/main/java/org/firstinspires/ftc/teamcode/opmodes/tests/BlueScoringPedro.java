package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LauncherConfig;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FieldPoses;
import org.firstinspires.ftc.teamcode.pedro.PedroLog;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

// PoC: BlueScoring on Pedro Pathing. Pedro drives the motors and the Pinpoint, so this skips Robot.
@Autonomous(name = "BlueScoringPedro", group = "Tests")
public class BlueScoringPedro extends LinearOpMode {

    private Telemetry tele;
    private Follower follower;
    private Launcher launcher;
    private Intake intake;
    private Vision vision;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private String lastShot = "none";

    // main entry point for the autonomous opmode
    @Override
    public void runOpMode() {
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // create and configure the Pedro follower
        follower = Constants.create(hardwareMap);
        // set the initial pose for the follower
        follower.setPose(FieldPoses.BLUE_SCORING_START);

        tele.update();

        // wait for the start of the match and reset the match timer
        waitForStart();
        matchTimer.reset();

        if (opModeIsActive()) {
            // wait for the auto start delay before initializing subsystems
            waitSeconds(RobotConstants.AUTO_START_DELAY_SECONDS);
            initStart();
            runRoutine();
        }

        initStop();
        tele.update();
    }

    // create and initialize subsystems at the start of the match
    private void initStart() {
        PedroLog.log(tele, "Start pose", PedroLog.format(follower.pose()));
        launcher = new Launcher(hardwareMap, tele);
        intake = new Intake(hardwareMap, tele);
        vision = new Vision(hardwareMap, tele);
        launcher.init();
        intake.init();
        vision.init();
        launcher.useAutoValues();
    }

    // stop and clean up subsystems at the end of the match
    private void initStop() {
        follower.stop();
        launcher.stop();
        intake.stop();
        vision.stop();
        PedroLog.log(tele, "End pose", PedroLog.format(follower.pose()));
    }

    private void runRoutine() {
        // start spinup immediately
        launcher.spinUp();
        // drive up, spin up, check the hive
        followPath("scan", line(FieldPoses.BLUE_SCORING_START, FieldPoses.BLUE_SCAN));

        // tipped: shoot. not tipped: back up a little
        Pose parkFrom;
        if (scanForTip(RobotConstants.TAG_BLUE_SCORING_MIN, RobotConstants.TAG_BLUE_SCORING_MAX)) {
            followPath("to shoot", line(FieldPoses.BLUE_SCAN, FieldPoses.BLUE_SHOOT));
            shoot(3);
            parkFrom = FieldPoses.BLUE_SHOOT;
        } else {
            // not tipped: back up a little
            followPath("back up", line(FieldPoses.BLUE_SCAN, FieldPoses.BLUE_NOT_TIPPED));
            parkFrom = FieldPoses.BLUE_NOT_TIPPED;
        }

        // leave and park
        followPath("park", Paths.path(
                line(parkFrom, FieldPoses.BLUE_PARK_TURN),
                line(FieldPoses.BLUE_PARK_TURN, FieldPoses.BLUE_PARK_LANE),
                line(FieldPoses.BLUE_PARK_LANE, FieldPoses.BLUE_PARK)));
    }

    // steps

    // straight line, heading goes from the first pose to the second
    // create a straight line path from one pose to another
    private Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }

    // follow a path until it ends or time runs out
    private void followPath(String name, Path path) {
        PedroLog.log(tele, "Path", name + " from " + PedroLog.format(follower.pose()));

        // start following the path
        // path is the trajectory the robot will follow
        follower.follow(path);
        while (!shouldStop() && follower.isBusy()) {
            tick();
        }
        PedroLog.log(tele, "Path done", name + " at " + PedroLog.format(follower.pose()));
    }

    private void waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (!shouldStop() && timer.seconds() < seconds) {
            tick();
        }
    }

    // same as AutoBase.scanForTip
    private boolean scanForTip(int minId, int maxId) {
        // start a timer for scanning
        // keep checking until the timer runs out or the hive is tipped
        // if the hive is tipped, return true immediately; takes april tag IDs into account
        // tick each loop to update subsystems and telemetry
        ElapsedTime timer = new ElapsedTime();
        while (!shouldStop() && timer.seconds() < LauncherConfig.SCAN_TIME) {
            if (vision.isHiveTipped(minId, maxId)) return true;
            tele.addData("Scan", "%.1fs left", LauncherConfig.SCAN_TIME - timer.seconds());
            tick();
        }
        return false;
    }

    // same as AutoBase.shoot
    // this one is for testing
    private void shoot(int count) {
        // turn intake on immediately
        intake.on();

        // loop that shoots the specified number of times, waiting for the launcher to be ready each time
        for (int i = 0; i < count && !shouldStop() && !timeToPark(); i++) {
            // starts a new timer to wait for the launcher to be ready
            // isReady() checks if the launcher has reached the desired velocity
            ElapsedTime readyTimer = new ElapsedTime();
            while (!shouldStop() && !timeToPark() && !launcher.isReady()
                    && readyTimer.seconds() < LauncherConfig.READY_TIMEOUT) {
                tick();
            }
            // if the launcher is not ready after the timeout, proceed anyway
            // if it's time to park, break out of the loop
            if (timeToPark()) break;
            lastShot = String.format("Shot %d: %s at %.0f, waited %.1fs", i + 1,
                    launcher.isReady() ? "READY" : "TIMEOUT", launcher.getVelocity(), readyTimer.seconds());
            PedroLog.log(tele, "Shot", lastShot);

            launcher.feed();
            waitSeconds(LauncherConfig.FEED_HOLD_TIME);
            launcher.retractFeeder();
            waitSeconds(LauncherConfig.FEED_PAUSE_TIME);
        }

        launcher.stopFlywheel();
        intake.off();
    }

    // helpers

    private double secondsLeft() {
        return RobotConstants.AUTO_PERIOD_SECONDS - matchTimer.seconds();
    }

    private boolean timeToPark() {
        return secondsLeft() <= LauncherConfig.PARK_TIME;
    }

    private boolean shouldStop() {
        return !opModeIsActive() || matchTimer.seconds() >= RobotConstants.AUTO_OVERDRIVE_SECONDS;
    }

    // one loop pass, the only telemetry.update()
    private void tick() {
        follower.update();
        launcher.update();
        intake.update();
        vision.update();
        tele.addData("Match time", "%.1f (%.1f left)", matchTimer.seconds(), secondsLeft());
        tele.addData("Pose", PedroLog.format(follower.pose()));
        tele.addData("Last shot", lastShot);
        tele.update();
    }
}
