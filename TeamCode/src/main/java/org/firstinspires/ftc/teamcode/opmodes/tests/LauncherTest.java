package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;



@TeleOp(name="Launcher Test", group="test")
@Config
public class LauncherTest extends LinearOpMode {

    private DcMotorEx launcher;
    private Servo feeder;

    public static PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients();
    public static double LAUNCHER_TARGET_VEL = 0;
    public static double LAUNCHER_TOLERANCE = 50;
    private boolean WITHIN_TOLERANCE = false;

    @Override
    public void runOpMode() {
        launcher  = hardwareMap.get(DcMotorEx.class, "launcher");
        feeder = hardwareMap.get(Servo.class, "feeder");
        telemetry = FtcDashboard.getInstance().getTelemetry();
        waitForStart();

        while (opModeIsActive()) {

            WITHIN_TOLERANCE = (Math.abs(LAUNCHER_TARGET_VEL - launcher.getVelocity()) >= LAUNCHER_TOLERANCE );

            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
            launcher.setVelocity(LAUNCHER_TARGET_VEL);

            if (gamepad1.right_trigger >= 0.3 && WITHIN_TOLERANCE) {
                feeder.setPosition(0.04);
            } else {
                feeder.setPosition(0.3);
            }

            telemetry.addData("Within Launch Tolerance?", WITHIN_TOLERANCE);
            telemetry.addData("Velocity", launcher.getVelocity());
            telemetry.update();
        }
    }
}
