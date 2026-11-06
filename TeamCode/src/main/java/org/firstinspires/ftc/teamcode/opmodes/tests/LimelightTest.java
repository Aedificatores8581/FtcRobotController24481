package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")

public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight;
    private Servo servo;

    private double servopos = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        servo = hardwareMap.get(Servo.class, "limelightservo");

        String tipped = "null";

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Telemetry telemetry = dashboard.getTelemetry();

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        dashboard.startCameraStream(limelight, 10);
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        servopos = 0.5;
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                servopos += 0.001;
            }
            if (gamepad1.dpad_down) {
                servopos -= 0.001;
            }

            servopos = Math.min(Math.max(servopos, 0.1), 0.9);

            servo.setPosition(servopos);

            telemetry.addData("servo position", servopos);
            telemetry.update();
        }
        limelight.stop();
    }
}
