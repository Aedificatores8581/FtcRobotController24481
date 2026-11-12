package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.List;

@TeleOp(name = "AimTestOdom", group = "Test")
@Config
public class AimTestOdom extends LinearOpMode {

    private DcMotorEx launcher;
    private DcMotor intake;

    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;

    private Servo feeder;

    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    // ==========================================
    // DASHBOARD SETTINGS
    // ==========================================

    public static PIDFCoefficients Pidf =
            new PIDFCoefficients(120, 0.67, 12, 150);

    // Base launcher velocity
    public static double vel = 1150;

    // Launcher velocity adjustment
    // Ticks/second per degree of Limelight vertical error
    public static double launcherVelocityGain = 8.0;

    // Maximum velocity adjustment from base velocity
    public static double maxLauncherVelocityChange = 100;

    // Launcher velocity safety limits
    public static double minLauncherVelocity = 800;
    public static double maxLauncherVelocity = 1500;

    // Rotation aiming gain
    public static double yawGain = 0.025;

    // Odometry rotation damping
    public static double rotationVelocityGain = 0.12;

    // Automatic rotation limit
    public static double maxAutoYaw = 0.35;

    // Deadbands
    public static double axialDeadband = 1.0;
    public static double yawDeadband = 0.7;

    // ==========================================
    // DRIVE VARIABLES
    // ==========================================

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    double axial;
    double lateral;
    double yaw;

    boolean intakeActive = false;

    // ==========================================
    // PINPOINT VELOCITY
    // ==========================================

    double forwardVelocity;
    double sidewaysVelocity;
    double angularVelocity;

    // ==========================================
    // AIMING VARIABLES
    // ==========================================

    String[] clusters = {
            "Red scoring",
            "Red Audience",
            "Blue Scoring",
            "Blue Audience"
    };

    double aimYaw = 0;
    double aimAxial = 0;

    // Launcher adjustment
    double autoLauncherVelocity = 0;
    double targetLauncherVelocity = 1150;

    // Automatic rotation
    double autoYaw = 0;

    int selectedCluster = -1;

    boolean targetFound = false;

    // ==========================================
    // MAIN OP MODE
    // ==========================================

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(
                telemetry,
                dashboard.getTelemetry()
        );

        // ==========================================
        // HARDWARE INITIALIZATION
        // ==========================================

        launcher = hardwareMap.get(
                DcMotorEx.class, "launcher");

        intake = hardwareMap.get(
                DcMotor.class, "intake");

        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        feeder = hardwareMap.get(
                Servo.class, "feeder");

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint");

        // ==========================================
        // MOTOR CONFIGURATION
        // ==========================================

        launcher.setDirection(DcMotor.Direction.REVERSE);

        launcher.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);

        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                Pidf
        );

        intake.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        // ==========================================
        // PINPOINT CONFIGURATION
        // ==========================================

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods
                        .goBILDA_4_BAR_POD
        );

        pinpoint.update();

        // ==========================================
        // LIMELIGHT CONFIGURATION
        // ==========================================

        feeder.setPosition(0.04);

        limelight.pipelineSwitch(0);
        limelight.start();

        dashboard.startCameraStream(limelight, 10);

        // Initial launcher velocity

        telemetry.addLine("AimTestOdom Ready");
        telemetry.update();

        waitForStart();

        // ==========================================
        // MAIN LOOP
        // ==========================================

        while (opModeIsActive()) {

            // Reset automatic aiming outputs every loop.
            // This prevents old corrections from remaining active.

            targetLauncherVelocity = vel;
            autoLauncherVelocity = 0;
            autoYaw = 0;

            // ==========================================
            // LAUNCHER PIDF
            // ==========================================

            launcher.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    Pidf
            );

            // ==========================================
            // FEEDER
            // ==========================================

            if (gamepad1.right_trigger >= 0.3) {
                feeder.setPosition(0.04);
            } else {
                feeder.setPosition(0.3);
            }

            if (gamepad1.dpad_down) {
                feeder.setPosition(
                        Math.max(
                                feeder.getPosition() - 0.01,
                                0
                        )
                );
            }

            if (gamepad1.dpad_up) {
                feeder.setPosition(
                        Math.min(
                                feeder.getPosition() + 0.01,
                                1
                        )
                );
            }

            // ==========================================
            // INTAKE
            // ==========================================

            if (gamepad1.rightBumperWasPressed()) {
                intakeActive = !intakeActive;
            }

            intake.setPower(intakeActive ? 0.8 : 0);

            // ==========================================
            // PINPOINT VELOCITY UPDATE
            // ==========================================

            pinpoint.update();

            forwardVelocity =
                    pinpoint.getVelX(DistanceUnit.MM) / 1000.0;

            sidewaysVelocity =
                    pinpoint.getVelY(DistanceUnit.MM) / 1000.0;

            angularVelocity =
                    pinpoint.getHeadingVelocity(
                            UnnormalizedAngleUnit.RADIANS
                    );

            // ==========================================
            // MANUAL DRIVE
            // ==========================================

            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            targetFound = false;
            selectedCluster = -1;

            // ==========================================
            // LIMELIGHT TARGET DETECTION
            // ==========================================

            LLResult result = limelight.getLatestResult();

            if (gamepad1.left_trigger > 0.3
                    && result != null
                    && result.isValid()) {

                double[] clusterYaw = new double[4];
                double[] clusterAxial = new double[4];

                int[] clusterCount = new int[4];

                List<LLResultTypes.FiducialResult> fiducials =
                        result.getFiducialResults();

                // Group AprilTags

                for (LLResultTypes.FiducialResult fr : fiducials) {

                    int id = fr.getFiducialId();

                    if (id >= 30 && id <= 33) {

                        clusterCount[0]++;
                        clusterYaw[0] += fr.getTargetXDegrees();
                        clusterAxial[0] += fr.getTargetYDegrees();

                    } else if (id >= 34 && id <= 37) {

                        clusterCount[1]++;
                        clusterYaw[1] += fr.getTargetXDegrees();
                        clusterAxial[1] += fr.getTargetYDegrees();

                    } else if (id >= 38 && id <= 41) {

                        clusterCount[2]++;
                        clusterYaw[2] += fr.getTargetXDegrees();
                        clusterAxial[2] += fr.getTargetYDegrees();

                    } else if (id >= 42 && id <= 45) {

                        clusterCount[3]++;
                        clusterYaw[3] += fr.getTargetXDegrees();
                        clusterAxial[3] += fr.getTargetYDegrees();
                    }
                }

                // ==========================================
                // SELECT CLUSTER
                // ==========================================

                int highest = 0;
                double closestToZero = Double.MAX_VALUE;

                for (int i = 0; i < 4; i++) {

                    if (clusterCount[i] == 0) {
                        continue;
                    }

                    double averageYaw =
                            clusterYaw[i] / clusterCount[i];

                    if (clusterCount[i] > highest
                            || (clusterCount[i] == highest
                            && Math.abs(averageYaw) < closestToZero)) {

                        highest = clusterCount[i];
                        closestToZero = Math.abs(averageYaw);
                        selectedCluster = i;
                    }
                }

                // ==========================================
                // CALCULATE AVERAGE OFFSETS
                // ==========================================

                if (selectedCluster != -1) {

                    aimYaw =
                            clusterYaw[selectedCluster]
                                    / clusterCount[selectedCluster];

                    aimAxial =
                            clusterAxial[selectedCluster]
                                    / clusterCount[selectedCluster];

                    targetFound = true;
                    // ==========================================
                    // AUTOMATIC AIMING
                    // ==========================================



                        // Apply target deadbands

                        double axialError =
                                Math.abs(aimAxial) < axialDeadband
                                        ? 0 : aimAxial;

                        double yawError =
                                Math.abs(aimYaw) < yawDeadband
                                        ? 0 : aimYaw;

                        // ==========================================
                        // LAUNCHER VELOCITY ADJUSTMENT
                        // ==========================================

                        axial = ((axialError-1.5)/-20);


                        // ==========================================
                        // AUTOMATIC ROTATION
                        // ==========================================

                        autoYaw =
                                -yawError * yawGain
                                        - angularVelocity
                                        * rotationVelocityGain;

                        // Limit rotation

                        autoYaw = Math.max(
                                -maxAutoYaw,
                                Math.min(maxAutoYaw, autoYaw)
                        );

                        // ==========================================
                        // AUTOMATIC DRIVE
                        // ==========================================

                        // No forward/backward movement.
                        // No strafing.
                        // Only rotation is allowed.

                        yaw = -autoYaw;


                    // ==========================================
                    // AIMING TELEMETRY
                    // ==========================================

                    telemetry.addData(
                            "Selected Cluster",
                            clusters[selectedCluster]
                    );

                    telemetry.addData("Aim Yaw", aimYaw);
                    telemetry.addData("Aim Axial", aimAxial);

                } else {

                    telemetry.addData(
                            "Aim Error",
                            "No recognized AprilTags"
                    );
                }
            }

            // ==========================================
            // APPLY LAUNCHER VELOCITY
            // ==========================================

            launcher.setVelocity(targetLauncherVelocity);

            // ==========================================
            // DRIVE CALCULATIONS
            // ==========================================

            frontLeftPower = axial + lateral + yaw;

            frontRightPower = axial - lateral - yaw;

            backLeftPower = axial - lateral + yaw;

            backRightPower = axial + lateral - yaw;

            // Normalize drive powers

            double max = JavaUtil.maxOfList(
                    JavaUtil.createListWith(
                            Math.abs(frontLeftPower),
                            Math.abs(frontRightPower),
                            Math.abs(backLeftPower),
                            Math.abs(backRightPower)
                    )
            );

            if (max > 1) {

                frontLeftPower /= max;
                frontRightPower /= max;

                backLeftPower /= max;
                backRightPower /= max;
            }

            // ==========================================
            // SEND DRIVE POWERS
            // ==========================================

            FL.setPower(frontLeftPower);
            FR.setPower(frontRightPower);

            BL.setPower(backLeftPower);
            BR.setPower(backRightPower);

            // ==========================================
            // TELEMETRY
            // ==========================================

            telemetry.addData(
                    "Launcher Base Velocity",
                    vel
            );

            telemetry.addData(
                    "Launcher Target Velocity",
                    targetLauncherVelocity
            );

            telemetry.addData(
                    "Launcher Actual Velocity",
                    launcher.getVelocity()
            );

            telemetry.addData(
                    "Launcher Velocity Adjustment",
                    autoLauncherVelocity
            );

            telemetry.addData(
                    "Launcher Current (mA)",
                    launcher.getCurrent(CurrentUnit.MILLIAMPS)
            );

            telemetry.addData(
                    "Intake Active",
                    intakeActive
            );

            telemetry.addData(
                    "Feeder Position",
                    feeder.getPosition()
            );

            telemetry.addData(
                    "Target Found",
                    targetFound
            );

            telemetry.addData(
                    "Forward Velocity (m/s)",
                    forwardVelocity
            );

            telemetry.addData(
                    "Sideways Velocity (m/s)",
                    sidewaysVelocity
            );

            telemetry.addData(
                    "Angular Velocity (rad/s)",
                    angularVelocity
            );

            telemetry.addData(
                    "Automatic Yaw",
                    autoYaw
            );

            telemetry.addData(
                    "Automatic Launcher Adjustment",
                    autoLauncherVelocity
            );

            telemetry.update();
        }
    }
}