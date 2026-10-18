package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="PIDF Testing")
@Config
public class PIDF_Testing extends LinearOpMode {

        public static PIDFCoefficients PIDFSet = new PIDFCoefficients(0.1, 0.1, 0, 13.1);
        public static double targetVel = 1150;

        FtcDashboard dashboard = FtcDashboard.getInstance();
    public void runOpMode() {
        DcMotorEx motor;

        motor = hardwareMap.get(DcMotorEx.class, "test");
        //Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //start the stuff I guess
        //dashboard.startCameraStream(limelight, 0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Detect if limelight, then send it to dashboard if so

        try {
            Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
            dashboard.startCameraStream(limelight, 0);
            telemetry.addData("Limelight", "Found");
        } catch (Exception e) {
            telemetry.addData("Limelight", "Not Found");
        }

        double lastVel = 0.0;

        double tolerance = 5.0;

        boolean toggled = false;

        waitForStart();

        while (opModeIsActive()) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDFSet);

            if (toggled) {
                telemetry.addData("Toggle", "On");
                motor.setVelocity(targetVel);
            } else {
                telemetry.addData("Toggle","Off");
                motor.setVelocity(0);
            }

            if (gamepad1.aWasPressed()) {
                if (toggled){
                    toggled = false;
                } else {
                    toggled = true;
                }
            }

            double currentVel = motor.getVelocity();

            if (Math.abs(currentVel) < tolerance) {
                telemetry.addData("Motor Status", "Stopped");
            } else {
                double velChange = Math.abs(currentVel) - Math.abs(lastVel);

                if (velChange > tolerance) {
                    telemetry.addData("Motor Status", "Speeding Up");
                } else if (velChange < -tolerance) {
                    telemetry.addData("Motor Status", "Slowing Down");
                } else {
                    telemetry.addData("Motor Status", "At Speed");
                }
            }

            lastVel = currentVel;
            telemetry.addData("CurrentVelocity", currentVel);
            telemetry.update();
        }
    }
}

