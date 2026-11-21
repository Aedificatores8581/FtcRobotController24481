package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoBase;

/*
 * PEDROTEST - Autonomous routine using 9 movement points.
 *
 *  - Explicit step tracking ensures every pose is executed strictly ONCE.
 *  - Calculates relative movement from start pose (0, 0, 0).
 *  - Smoothly turns and drives without swerving.
 *  - Begins by running the aimtester shooting sequence.
 *  - Movement lines 1 & 2.
 *  - After point 2: retracts feeder (no feed) and turns ON intake.
 *  - Movement lines 3 & 4 (intake runs while picking up balls).
 *  - After line 4: turns OFF intake.
 *  - Movement line 5.
 *  - After movement line 5: shoots collected balls for 5 seconds.
 *  - Continues with movement lines 6 through 9.
 */
@Autonomous(name = "pedrotest", group = "Test")
public class pedrotest extends AutoBase {

    public static class Pose {
        public final double x, y, heading;

        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    // Poses generated from coordinates
    public final Pose start  = new Pose(55.5000, 8.5000, 90.0000);
    public final Pose point1 = new Pose(55.4259, 35.6201, 96.0882);
    public final Pose point2 = new Pose(9.2449, 36.3079, 179.1468);
    public final Pose point3 = new Pose(7.7856, 8.3322, -92.9860);
    public final Pose point4 = new Pose(58.2092, 7.4060, -1.0523);
    public final Pose point5 = new Pose(58.0440, 13.0153, 91.6872);
    public final Pose point6 = new Pose(56.8534, 35.7573, 92.9968);
    public final Pose point7 = new Pose(26.1734, 36.7492, 178.1483);
    public final Pose point8 = new Pose(22.9370, 98.5705, 92.9967);
    public final Pose point9 = new Pose(1.9873, 99.0138, 178.7878);

    private int completedStep = 0; // Tracks finished steps so no pose is repeated

    @Override
    protected void runRoutine() {
        completedStep = 0;

        // 1. Shoot pre-loaded balls at beginning (using working aimtester sequence)
        shootAimtesterStyle(3.0);

        // Movement line 1
        driveToStep(1, "Point 1", point1);

        // Movement line 2 (retract feeder, no feed)
        driveToStep(2, "Point 2", point2);
        robot.launcher.retractFeeder();

        // Movement line 3 (turn ON intake for line 3)
        robot.intake.on();
        driveToStep(3, "Point 3", point3);

        // Movement line 4
        driveToStep(4, "Point 4", point4);
        robot.intake.off();

        // Movement line 5
        driveToStep(5, "Point 5", point5);

        // After movement line 5: shoot collected balls for 5 seconds
        shootAimtesterStyle(5.0);

        // Movement lines 6 through 9
        driveToStep(6, "Point 6", point6);
        driveToStep(7, "Point 7", point7);
        driveToStep(8, "Point 8", point8);
        driveToStep(9, "Point 9", point9);
    }

    /** Converts absolute Pedro Pathing target into relative pose based on start (0, 0, 0). */
    private Pose toRelativePose(Pose target) {
        return new Pose(
                target.x - start.x,
                target.y - start.y,
                target.heading - start.heading
        );
    }

    /** Drives to a target pose cleanly and marks step as completed so it is never re-executed. */
    private void driveToStep(int stepIndex, String stepName, Pose absoluteTarget) {
        if (completedStep >= stepIndex) {
            return; // Skip if step was already completed
        }

        telemetry.addData("Auto Pathing", "Executing Step %d / 9: %s", stepIndex, stepName);
        robot.update();

        Pose target = toRelativePose(absoluteTarget);

        double maxPower = RobotConstants.AUTO_POWER;
        ElapsedTime segmentTimer = new ElapsedTime();
        double timeoutSeconds = 6.0;

        while (opModeIsActive() && segmentTimer.seconds() < timeoutSeconds) {
            robot.update();

            double currentX = robot.odo.getXInches();
            double currentY = robot.odo.getYInches();
            double currentHeading = robot.odo.getHeadingDegrees();

            double dx = target.x - currentX;
            double dy = target.y - currentY;
            double distance = Math.hypot(dx, dy);

            double headingError = target.heading - currentHeading;

            // Normalize angle error to [-180, 180]
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            if (distance < 1.5 && Math.abs(headingError) < 4.0) {
                break; // Arrived at target
            }

            // Transform field offsets into robot frame
            double headingRad = Math.toRadians(currentHeading);
            double robotForward = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
            double robotStrafe  = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);

            // Proportional velocity scaling
            double fwdPower = Math.max(-maxPower, Math.min(maxPower, robotForward * 0.10));
            double strPower = Math.max(-maxPower, Math.min(maxPower, robotStrafe * 0.10));
            double turnPower = Math.max(-maxPower, Math.min(maxPower, headingError * 0.015));

            robot.drive.drive(fwdPower, strPower, turnPower);
        }

        robot.drive.drive(0, 0, 0);
        robot.update();

        // Mark step as permanently completed
        completedStep = stepIndex;
        telemetry.addData("Auto Pathing", "Completed Step %d / 9: %s", stepIndex, stepName);
        robot.update();

        sleep(RobotConstants.AUTO_PAUSE_MS);
    }

    /** Working shooting sequence built in aimtester.java. */
    private void shootAimtesterStyle(double intakeWaitSeconds) {
        robot.launcher.spinUp();

        // Wait in a loop until launcher is ready
        while (opModeIsActive() && !robot.launcher.isReady()) {
            robot.update();
        }

        robot.launcher.feed();
        waitSeconds(1);
        robot.intake.on();
        waitSeconds(intakeWaitSeconds);
        robot.update();

        robot.launcher.stopFlywheel();
        robot.intake.off();
    }
}
