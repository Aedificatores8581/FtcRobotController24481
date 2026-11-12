package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FieldPoses;
import org.firstinspires.ftc.teamcode.pedro.PedroLog;

// shows the Pedro pose while you move the robot by hand, for finding field positions
// A: set the pose back to the blue scoring start. B: write the current pose to the log
@TeleOp(name = "PoseReadout", group = "Tests")
public class PoseReadout extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Follower follower = Constants.create(hardwareMap);
        follower.setPose(FieldPoses.BLUE_SCORING_START);

        tele.addLine("Robot at start tile, keep still until INIT finishes");
        tele.update();
        waitForStart();

        boolean lastA = false;
        boolean lastB = false;
        while (opModeIsActive()) {
            follower.update();

            if (gamepad1.a && !lastA) follower.setPose(FieldPoses.BLUE_SCORING_START);
            if (gamepad1.b && !lastB) PedroLog.log(tele, "Marked", PedroLog.format(follower.pose()));
            lastA = gamepad1.a;
            lastB = gamepad1.b;

            tele.addData("Pose", PedroLog.format(follower.pose()));
            tele.addLine("A: reset to start   B: log pose");
            tele.update();
        }
    }
}
