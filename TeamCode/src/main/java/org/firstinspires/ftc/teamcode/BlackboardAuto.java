package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * Shows how an autonomous OpMode can leave a note for the TeleOp that runs after it.
 *
 * The "blackboard" is a shared notepad (a key -> value map) that every OpMode can read and
 * write. It survives between OpModes, so auto can save things teleOp wants to know, such as
 * which alliance we are on. It does NOT survive a robot restart or an app re-deploy.
 *
 *     During INIT:  press left bumper for RED, right bumper for BLUE
 *     Press START:  the choice is saved, then the OpMode ends
 *
 * Then run TeleOp Basic: it displays the saved alliance on the Driver Station.
 *
 * Rules: use the same key text in both places (we use a constant so a typo can't sneak in),
 * and read back the same type you stored (we store a String, so we read a String).
 */
@Autonomous(name = "Blackboard Auto (pick alliance)", group = "Auto")
public class BlackboardAuto extends LinearOpMode {

    private static final String TAG = "BlackboardAuto";

    /** The note's label. TeleOpBasic uses this same constant to find it. */
    public static final String ALLIANCE_KEY = "Alliance";

    @Override
    public void runOpMode() {
        String alliance = "RED";

        // Runs repeatedly during INIT until the driver presses START (or STOP).
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.left_bumper)  alliance = "RED";
            if (gamepad1.right_bumper) alliance = "BLUE";

            telemetry.addLine("Left bumper = RED, right bumper = BLUE");
            telemetry.addData("Alliance", alliance);
            telemetry.update();
        }
        if (isStopRequested()) return;

        blackboard.put(ALLIANCE_KEY, alliance);
        RobotLog.ii(TAG, "Saved alliance to blackboard: %s", alliance);

        telemetry.addData("Saved", alliance);
        telemetry.addLine("Now run TeleOp Basic to see it.");
        telemetry.update();
        sleep(2000); // leave the message up for two seconds
    }
}
