package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@TeleOp(name = "CoolTeleop", group = "fun")
@Config
public class CoolTeleop extends LinearOpMode {

    private DcMotorEx launcher;
    private DcMotor intake;

    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;

    private Servo feeder;
    private Servo limelightServo;

    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    // ==========================================
    // DASHBOARD SETTINGS
    // ==========================================

    public static PIDFCoefficients Pidf =
            new PIDFCoefficients(120, 0.67, 12, 150);

    public static double vel = 1150;
    public static double position = 0.3;

    // ==========================================
    // AUTOMATIC DRIVE SETTINGS
    // ==========================================
    public static double axialAdjust = 2.15;
    public static double axialForwardGain = 0.03;
    public static double maxAutoAxial = 0.5;

    public static double yawGain = 0.025;
    public static double rotationVelocityGain = 0.12;
    public static double maxAutoYaw = 0.5;

    public static double axialDeadband = 1.0;
    public static double yawDeadband = 0.7;

    public static final int SMOOTHING_WINDOW = 10;
    public static double SMOOTHING_ALPHA = 0.2;
    public static double yawOffsetDegrees = -9.0;

    private double[] aimYawHistory = new double[SMOOTHING_WINDOW];
    private double[] aimAxialHistory = new double[SMOOTHING_WINDOW];

    private int smoothingIndex = 0;
    private int previousSmoothingCluster = -1;
    private int smoothingCount = 0;

    private double smoothedAimYaw = 0;
    private double smoothedAimAxial = 0;

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

    double targetLauncherVelocity = 1150;

    double autoYaw = 0;
    double yawError = 0;
    double axialError = 0;
    boolean targetFound = false;

    int selectedCluster = -1;

    LLResult result;
    int[] clusterCount = new int[4];
    double[] clusterYaw = new double[4];
    List<LLResultTypes.FiducialResult> fiducials = new ArrayList<>();
    StringBuilder detectedIDs = new StringBuilder();

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

        initializeHardware();
        configureMotors();
        configurePinpoint();
        configureLimelight();

        telemetry.addLine("AimTestOdom Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateLauncherPidf();
            updateFeeder();
            updateIntake();
            updatePinpointVelocity();
            updateManualDrive();
            detectLimelightTargets();
            groupAprilTags();
            selectCluster();
            updateSelectedClusterAiming();
            checkAutoAimingTarget();
            updateLauncher();
            calculateDrivePowers();
            sendDrivePowers();
            fieldDraw();
            updateTelemetry();
        }
    }

    private void initializeHardware() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");

        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        feeder = hardwareMap.get(Servo.class, "feeder");
        limelightServo = hardwareMap.get(Servo.class, "limelightservo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint"
        );
    }

    private void configureMotors() {
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, Pidf
        );

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void configurePinpoint() {
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.update();
    }

    private void configureLimelight() {
        feeder.setPosition(0.04);

        limelight.pipelineSwitch(0);
        limelight.start();

        FtcDashboard.getInstance().startCameraStream(limelight, 10);
    }

    private void updateLauncherPidf() {
        targetLauncherVelocity = vel;
        autoYaw = 0;
        yawError = 0;
        axialError = 0;

        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, Pidf
        );
    }

    private void updateFeeder() {
        if (gamepad1.left_trigger >= 0.3) {
            feeder.setPosition(0.04);
        } else {
            feeder.setPosition(0.3);
        }

        if (gamepad1.dpad_down) {
            feeder.setPosition(
                    Math.max(feeder.getPosition() - 0.01, 0)
            );
        }

        if (gamepad1.dpad_up) {
            feeder.setPosition(
                    Math.min(feeder.getPosition() + 0.01, 1)
            );
        }
    }

    private void updateIntake() {
        if (gamepad1.rightBumperWasPressed()) {
            intakeActive = !intakeActive;
        }

        intake.setPower(intakeActive ? 0.8 : 0);
    }

    private void updatePinpointVelocity() {
        pinpoint.update();

        forwardVelocity =
                pinpoint.getVelX(DistanceUnit.MM) / 1000.0;

        sidewaysVelocity =
                pinpoint.getVelY(DistanceUnit.MM) / 1000.0;

        angularVelocity =
                pinpoint.getHeadingVelocity(
                        UnnormalizedAngleUnit.RADIANS
                );
    }

    private void updateManualDrive() {
        axial = -gamepad1.left_stick_y;
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;

        targetFound = false;
        selectedCluster = -1;
    }

    private void detectLimelightTargets() {
        result = limelight.getLatestResult();

        clusterCount = new int[4];
        clusterYaw = new double[4];

        fiducials = new ArrayList<>();

        detectedIDs = new StringBuilder();

        if (result != null) {
            List<LLResultTypes.FiducialResult> detected =
                    result.getFiducialResults();

            if (detected != null) {
                fiducials = detected;
            }
        }
    }

    private void groupAprilTags() {
        for (LLResultTypes.FiducialResult fr : fiducials) {

            int id = fr.getFiducialId();

            detectedIDs.append(id).append(" ");

            int cluster;

            if (id >= 30 && id <= 33) {
                cluster = 0;
            } else if (id >= 34 && id <= 37) {
                cluster = 1;
            } else if (id >= 38 && id <= 41) {
                cluster = 2;
            } else if (id >= 42 && id <= 45) {
                cluster = 3;
            } else {
                cluster = -1;
            }

            if (cluster != -1) {
                clusterCount[cluster]++;
                clusterYaw[cluster] += fr.getTargetXDegrees();
            }
        }
    }

    private void selectCluster() {
        int highest = 0;
        double closestToZero = Double.MAX_VALUE;

        for (int i = 0; i < 4; i++) {

            if (clusterCount[i] == 0) {
                continue;
            }

            double averageYaw =
                    clusterYaw[i] / clusterCount[i];

            double distanceFromCenter = Math.abs(averageYaw);

            if (clusterCount[i] > highest ||
                    (clusterCount[i] == highest &&
                            distanceFromCenter < closestToZero)) {

                highest = clusterCount[i];
                closestToZero = distanceFromCenter;
                selectedCluster = i;
            }
        }
    }

    private void updateSelectedClusterAiming() {
        if (selectedCluster != -1) {

            List<Double> yawValues = new ArrayList<>();
            List<Double> axialValues = new ArrayList<>();

            for (LLResultTypes.FiducialResult fr : fiducials) {

                int id = fr.getFiducialId();

                boolean belongsToSelectedCluster =
                        (selectedCluster == 0 && id >= 30 && id <= 33) ||
                                (selectedCluster == 1 && id >= 34 && id <= 37) ||
                                (selectedCluster == 2 && id >= 38 && id <= 41) ||
                                (selectedCluster == 3 && id >= 42 && id <= 45);

                if (belongsToSelectedCluster) {
                    yawValues.add(fr.getTargetXDegrees());
                    axialValues.add(fr.getTargetYDegrees());
                }
            }

            Collections.sort(yawValues);

            int yawCount = yawValues.size();

            if (yawCount > 0) {

                if (yawCount % 2 == 0) {
                    aimYaw = (
                            yawValues.get(yawCount / 2 - 1) +
                                    yawValues.get(yawCount / 2)
                    ) / 2.0;

                } else {
                    aimYaw = yawValues.get(yawCount / 2);
                }
            }

            Collections.sort(axialValues);

            int axialCount = axialValues.size();

            if (axialCount > 0) {

                if (axialCount % 2 == 0) {
                    aimAxial = (
                            axialValues.get(axialCount / 2 - 1) +
                                    axialValues.get(axialCount / 2)
                    ) / 2.0;

                } else {
                    aimAxial = axialValues.get(axialCount / 2);
                }
            }

            targetFound = true;

            if (selectedCluster != previousSmoothingCluster) {
                smoothingIndex = 0;
                smoothingCount = 0;
                previousSmoothingCluster = selectedCluster;
            }

            aimYawHistory[smoothingIndex] = aimYaw;
            aimAxialHistory[smoothingIndex] = aimAxial;

            smoothingIndex = (smoothingIndex + 1) % SMOOTHING_WINDOW;

            if (smoothingCount < SMOOTHING_WINDOW) {
                smoothingCount++;
            }

            double yawSum = 0;
            double axialSum = 0;
            double totalWeight = 0;

            int newestIndex =
                    (smoothingIndex - 1 + SMOOTHING_WINDOW)
                            % SMOOTHING_WINDOW;

            for (int age = 0; age < smoothingCount; age++) {
                int index =
                        (newestIndex - age + SMOOTHING_WINDOW)
                                % SMOOTHING_WINDOW;

                double weight =
                        Math.pow(1.0 - SMOOTHING_ALPHA, age);

                yawSum += aimYawHistory[index] * weight;
                axialSum += aimAxialHistory[index] * weight;

                totalWeight += weight;
            }

            smoothedAimYaw = yawSum / totalWeight;
            smoothedAimAxial = axialSum / totalWeight;

            if (gamepad1.right_trigger >= 0.5) {

                yawError =
                        Math.abs(smoothedAimYaw) < yawDeadband
                                ? 0 : smoothedAimYaw;

                axialError =
                        Math.abs(smoothedAimAxial) < axialDeadband
                                ? 0 : smoothedAimAxial;

                autoYaw =
                        -yawError * yawGain
                                - angularVelocity * rotationVelocityGain;

                autoYaw = Math.max(
                        -maxAutoYaw,
                        Math.min(maxAutoYaw, autoYaw)
                );

                axial = (-axialError + axialAdjust) * axialForwardGain;

                axial = Math.max(
                        -maxAutoAxial,
                        Math.min(maxAutoAxial, axial)
                );

                yaw = -autoYaw;

                if ((Math.abs(axial) <= 0.05) && (Math.abs(yaw) <= 0.05)) {
                    intakeActive = true;
                    feeder.setPosition(0.04);
                }
            }
        } else {
            smoothingIndex = 0;
            smoothingCount = 0;

            previousSmoothingCluster = -1;

            smoothedAimYaw = 0;
            smoothedAimAxial = 0;
        }
    }

    private void checkAutoAimingTarget() {
        if (gamepad1.right_trigger >= 0.5 && !targetFound) {
            axial = 0;
            lateral = 0;
            yaw = 0;
        }
    }

    private void updateLauncher() {
        targetLauncherVelocity = vel;
        launcher.setVelocity(targetLauncherVelocity);
    }

    private void calculateDrivePowers() {
        frontLeftPower = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;

        backLeftPower = axial - lateral + yaw;
        backRightPower = axial + lateral - yaw;

        limelightServo.setPosition(position);

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
    }

    private void sendDrivePowers() {
        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);

        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
    }

    private static final double CAM_W = 320.0, CAM_H = 240.0;      // match pipeline resolution
    private static final double IMG_W = 144.0, IMG_H = 108.0;
    private static final double IMG_X = -IMG_W / 2, IMG_Y = -IMG_H / 2;
    private static final double FOV_H = 54.5, FOV_V = 42.0;         // verify for your camera

    private double mapX(double px) { return IMG_X + (px / CAM_W) * IMG_W; }
    private double mapY(double py) { return IMG_Y + (py / CAM_H) * IMG_H; }

    private void fieldDraw() {
        TelemetryPacket packet = new TelemetryPacket(false);
        Canvas canvas = packet.fieldOverlay();

        canvas.setRotation(Math.PI / 2);   // purely cosmetic now; adjust until the image is upright
        canvas.drawImage("http://192.168.43.1:5800", IMG_X, IMG_Y, IMG_W, IMG_H);

        if (targetFound && fiducials != null) {
            for (LLResultTypes.FiducialResult fr : fiducials) {
                List<List<Double>> corners = fr.getTargetCorners();
                if (corners == null || corners.size() != 4) continue;

                double[] xs = new double[4], ys = new double[4];
                for (int i = 0; i < 4; i++) {
                    xs[i] = mapX(corners.get(i).get(0));
                    ys[i] = mapY(corners.get(i).get(1));
                }
                canvas.setAlpha(0.25);
                canvas.setFill("green");
                canvas.fillPolygon(xs, ys);
                canvas.setAlpha(1.0);
                canvas.setStroke("lime");
                canvas.setStrokeWidth(1);
                canvas.strokePolygon(xs, ys);
            }

            // tx/ty (degrees from crosshair) -> pixel, using tan for correct projection
            double px = CAM_W / 2 + Math.tan(Math.toRadians(aimYaw))
                    / Math.tan(Math.toRadians(FOV_H / 2)) * (CAM_W / 2);
            double py = CAM_H / 2 - Math.tan(Math.toRadians(aimAxial))
                    / Math.tan(Math.toRadians(FOV_V / 2)) * (CAM_H / 2);
            double cx = mapX(px), cy = mapY(py);

            canvas.setAlpha(1.0);
            canvas.setStroke("red");
            canvas.setStrokeWidth(2);
            canvas.strokeCircle(cx, cy, 4);
            canvas.strokeLine(cx - 6, cy, cx + 6, cy);
            canvas.strokeLine(cx, cy - 6, cx, cy + 6);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }


    private void updateTelemetry() {
        telemetry.addData(
                "Limelight",
                "Result: %s | Tags: %d | IDs: %s",
                result != null,
                fiducials.size(),
                detectedIDs.toString()
        );

        telemetry.addData(
                "Cluster Counts",
                "RS: %d | RA: %d | BS: %d | BA: %d",
                clusterCount[0],
                clusterCount[1],
                clusterCount[2],
                clusterCount[3]
        );

        telemetry.addData(
                "Selected Cluster",
                selectedCluster == -1
                        ? "None"
                        : clusters[selectedCluster]
        );

        telemetry.addData(
                "Target Found",
                targetFound
        );

        telemetry.addData(
                "Aim Yaw",
                "Raw: %.2f | Smooth: %.2f | Error: %.2f",
                aimYaw,
                smoothedAimYaw,
                yawError
        );

        telemetry.addData(
                "Aim Axial",
                "Raw: %.2f | Smooth: %.2f | Error: %.2f",
                aimAxial,
                smoothedAimAxial,
                axialError
        );

        telemetry.addData(
                "Auto Drive",
                "Yaw: %.3f | Axial: %.3f",
                autoYaw,
                axial
        );

        telemetry.addData(
                "Pinpoint Velocity",
                "Forward: %.3f | Sideways: %.3f | Rotation: %.3f",
                forwardVelocity,
                sidewaysVelocity,
                angularVelocity
        );

        telemetry.addData(
                "Launcher",
                "Target: %.0f | Actual: %.0f | Current: %.0f mA",
                targetLauncherVelocity,
                launcher.getVelocity(),
                launcher.getCurrent(CurrentUnit.MILLIAMPS)
        );

        telemetry.addData(
                "Mechanisms",
                "Intake: %s | Feeder: %.2f",
                intakeActive,
                feeder.getPosition()
        );

        telemetry.addData(
                "Drive Powers",
                "FL: %.2f | FR: %.2f | BL: %.2f | BR: %.2f",
                frontLeftPower,
                frontRightPower,
                backLeftPower,
                backRightPower
        );

        telemetry.update();
    }
}
