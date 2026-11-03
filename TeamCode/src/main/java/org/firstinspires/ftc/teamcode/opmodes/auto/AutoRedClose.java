package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;

/* Ported from Rayden's red_down_close_load with Limelight Vision & Path branching. */
@Autonomous(name = "Red Close", group = "Main")
public class AutoRedClose extends AutoBase {

    @Override
    protected void runRoutine() {
        double p = RobotConstants.AUTO_POWER;

        // Step 1: Forward 20 inches
        driveInches(20, p);

        // Step 2: Limelight TipDetect scan (wait up to 8s for AprilTag cluster 42..45)
        ElapsedTime waitTimer = new ElapsedTime();
        boolean isTipped = false;

        while (opModeIsActive() && waitTimer.seconds() < 8.0) {
            robot.update(); // CRITICAL: Updates Limelight camera feed and telemetry on every frame!

            isTipped = robot.vision.TipDetect(42, 45);

            telemetry.addData("Scan Timer", "%.1fs remaining", 8.0 - waitTimer.seconds());
            telemetry.addData("Limelight Target", robot.vision.hasTarget() ? "YES" : "NO");
            telemetry.addData("Tip Detected", isTipped ? "TRUE" : "FALSE");

            if (isTipped) {
                break;
            }
            sleep(50);
        }

        double remainingSeconds = Math.max(0.0, 8.0 - waitTimer.seconds());

        // Branching Logic
        if (isTipped) {
            // Saw the camera tag! Run motors for remaining time left on the 8s timer
            if (remainingSeconds > 0) {
                driveSeconds(remainingSeconds, p);
            }
            // Then execute Path A (Reverse 8 IN & run leaveAndPark)
            driveInches(8, -p);
            leaveAndPark();
        } else if (waitTimer.seconds() >= 8.0) {
            // 8s limit reached -> Path A: Reverse 8 IN & run leaveAndPark
            driveInches(8, -p);
            leaveAndPark();
        } else {
            // Path B: Cleared / Upright -> Reverse 19 IN and stop
            driveInches(19, -p);
        }
    }

    private void leaveAndPark() {
        double p = RobotConstants.AUTO_POWER;

        turnToDegrees(-90, p);
        driveInches(41, p);
    }
}
