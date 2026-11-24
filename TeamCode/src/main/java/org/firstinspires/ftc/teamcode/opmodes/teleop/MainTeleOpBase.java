
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


/**
 * Shared TeleOp base class for Red and Blue alliances.
 *
 * Handles:
 * - Mecanum drive
 * - Limelight Python SnapScript targeting
 * - Automatic aiming and distance control
 * - Launcher, feeder, and intake
 * - Pinpoint odometry
 * - Limelight HUD communication
 *
 * Run MainTeleOpRed or MainTeleOpBlue.
 */
@Config
public abstract class MainTeleOpBase extends LinearOpMode {

    // =========================================================
    // ALLIANCE
    // =========================================================

    protected static final int ALLIANCE_RED = 1;
    protected static final int ALLIANCE_BLUE = 2;

    protected abstract int getAlliance();

    private static final String[] CLUSTERS = {
            "Red Scoring",
            "Red Audience",
            "Blue Scoring",
            "Blue Audience"
    };

    private boolean clusterAllowed(int cluster) {
        if (getAlliance() == ALLIANCE_RED) {
            return cluster == 0 || cluster == 1;
        }

        if (getAlliance() == ALLIANCE_BLUE) {
            return cluster == 2 || cluster == 3;
        }

        return true;
    }

    private String allianceName() {
        return getAlliance() == ALLIANCE_RED ? "RED" : "BLUE";
    }


    // =========================================================
    // DASHBOARD SETTINGS
    // =========================================================

    // Launcher
    public static PIDFCoefficients Pidf =
            new PIDFCoefficients(28, 1, 0, 13);

    private String order(int rank, String label) {
        StringBuilder prefix = new StringBuilder();
        for (int i = 0; i < rank; i++) {
            prefix.append('\u200C'); // Zero-Width Space
        }
        return prefix.toString() + label;
    }
    //This function allows FTC Dashboard to order the telemetry correctly

    public static double vel = 2000;
    public static double LAUNCH_INTAKE_SPEED = 0.5;
    public static double LAUNCH_VELOCITY_TOLERANCE = 40;

    public static int CORRECT_READINGS_BEFORE_SHOOT = 10;

    // Feeder / Limelight servo
    public static double position = 0.15;

    // Automatic axial positioning
    public static double AXIAL_TARGET_INCHES = 63.5;

    public static double AXIAL_P = 0.06;
    public static double AXIAL_I = 0.0;
    public static double AXIAL_D = 0.0;
    public static double AXIAL_F = 0.0;

    public static double MAX_AUTO_AXIAL = 0.35;

    public static double AXIAL_ERROR_TOL = 1.5;
    public static double YAW_ERROR_TOL = 1.5;

    // Automatic yaw
    public static double yawGain = 0.025;
    public static double rotationVelocityGain = 0.12;
    public static double maxAutoYaw = 0.35;

    // Limelight
    public static final int pythonPipeline = 3;
    public static final double maxStalenessMs = 150;


    // =========================================================
    // HARDWARE
    // =========================================================

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

    private FtcDashboard dashboard;


    // =========================================================
    // ROBOT STATE
    // =========================================================

    private boolean launcherActive = false;
    private boolean intakeActive = false;
    // Launch diagnostics
    private String launchBlockReason = "AUTO AIM OFF";

    private boolean launcherReady;
    private boolean axialAligned;
    private boolean yawAligned;

    // Drive commands
    private double axial;
    private double lateral;
    private double yaw;

    // Drive motor powers
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    // Pinpoint velocity
    private double forwardVelocity;
    private double sidewaysVelocity;
    private double angularVelocity;

    // Limelight targeting
    private double aimYaw;
    private double aimAxial;

    private double autoYaw;
    private double yawError;
    private double axialError;

    private boolean targetFound;
    private boolean aimLocked;

    private int selectedCluster = -1;

    // Axial PID state
    private double axialIntegral;
    private double previousAxialError;
    private long previousAxialTime;

    private int goodReadings = 0;


    // =========================================================
    // MAIN OP MODE
    // =========================================================

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(
                telemetry,
                dashboard.getTelemetry()
        );

        initRobot();

        telemetry.addLine("Limelight Aim " + allianceName() + " Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // props to chad gpt for making this easier to understand :P


            // Update robot localization and velocity.
            updateOdometry();
            // Handle launcher, feeder, and intake.
            launcherAndIntake();
            // Calculate automatic and manual drive. (includes auto aiming)
            drive();
            // Send robot information to the Limelight HUD.
            updateLimelightHUD();
            // Display robot information.
            updateTelemetry();
        }

        limelight.stop();
        dashboard.stopCameraStream();
    }


    // =========================================================
    // INITIALIZATION
    // =========================================================

    private void initRobot() {

        // Hardware mapping
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
                GoBildaPinpointDriver.class,
                "pinpoint"
        );


        // -------------------------
        // Launcher configuration
        // -------------------------

        launcher.setDirection(DcMotor.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                Pidf
        );


        // -------------------------
        // Intake configuration
        // -------------------------

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // -------------------------
        // Drive configuration
        // -------------------------

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // -------------------------
        // Pinpoint configuration
        // -------------------------

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.update();


        // -------------------------
        // Limelight configuration
        // -------------------------

        feeder.setPosition(0.04);

        limelight.pipelineSwitch(pythonPipeline);
        limelight.start();

        dashboard.startCameraStream(limelight, 10);
    }


    // =========================================================
    // LAUNCHER / MECHANISMS
    // =========================================================

    private void launcherAndIntake() {

        // Update PIDF continuously to allow Dashboard tuning.
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                Pidf
        );


        // -------------------------
        // Launcher toggle
        // X = Toggle launcher
        // -------------------------

        if (gamepad1.xWasPressed()) {
            launcherActive = !launcherActive;
        }

        launcher.setVelocity(launcherActive ? vel : 0);


        // -------------------------
        // Feeder control
        // Left trigger = Retract feeder
        // Otherwise = Extend feeder
        // D-pad = Fine adjustment
        // -------------------------

        if ((gamepad1.left_trigger >= 0.3) || (goodReadings >= CORRECT_READINGS_BEFORE_SHOOT)){
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


        // -------------------------
        // Intake toggle
        // Right bumper = Toggle intake
        // Triggers change intake speed
        // -------------------------

        if (gamepad1.rightBumperWasPressed()) {
            intakeActive = !intakeActive;
        }

        double intakePower = 0;

        if (intakeActive) {
            intakePower =
                    (gamepad1.right_trigger >= 0.5 ||
                            gamepad1.left_trigger >= 0.3)
                            ? LAUNCH_INTAKE_SPEED
                            : 0.8;
        }

        intake.setPower(intakePower);
    }


    // =========================================================
    // PINPOINT ODOMETRY
    // =========================================================

    private void updateOdometry() {

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


    // =========================================================
    // AUTOMATIC AIM
    // =========================================================

    private void autoAim() {

        targetFound = false;
        aimLocked = false;
        selectedCluster = -1;

        autoYaw = 0;
        yawError = 0;
        axialError = 0;

        launchBlockReason = "AUTO AIM OFF";

        axialAligned = false;
        yawAligned = false;

        launcherReady =
                Math.abs(launcher.getVelocity() - vel)
                        <= LAUNCH_VELOCITY_TOLERANCE;

        // Read Limelight
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            launchBlockReason = "NO LIMELIGHT RESULT";
            aimYaw = 0;
            aimAxial = 0;
            return;
        }

        double[] py = result.getPythonOutput();

        boolean fresh =
                result.getStaleness() < maxStalenessMs;

        if (!fresh) {
            launchBlockReason = "STALE LIMELIGHT DATA";
            aimYaw = 0;
            aimAxial = 0;
            return;
        }

        if (py == null || py.length < 7) {
            launchBlockReason = "INVALID PYTHON OUTPUT";
            aimYaw = 0;
            aimAxial = 0;
            return;
        }

        if (py[0] < 0.5) {
            launchBlockReason = "NO TARGET DETECTED";
            aimYaw = 0;
            aimAxial = 0;
            return;
        }

        // Validate cluster
        int cluster = (int) Math.round(py[3]);

        if (cluster < 0 ||
                cluster >= CLUSTERS.length ||
                !clusterAllowed(cluster)) {

            launchBlockReason = "INVALID / WRONG ALLIANCE TARGET";
            aimYaw = 0;
            aimAxial = 0;
            return;
        }

        selectedCluster = cluster;
        targetFound = true;

        aimYaw = py[1];
        aimAxial = py[5] * 39.37;

        // Automatic yaw
        yawError = aimYaw;

        autoYaw =
                -yawError * yawGain
                        - angularVelocity * rotationVelocityGain;

        autoYaw = Math.max(
                -maxAutoYaw,
                Math.min(maxAutoYaw, autoYaw)
        );

        // Auto aim disabled: still report target information.
        if (gamepad1.right_trigger < 0.5) {
            launchBlockReason = "AUTO AIM OFF";
            goodReadings = 0;
            return;
        }

        // Axial PID
        axialError = -(AXIAL_TARGET_INCHES - aimAxial);

        long currentTime = System.nanoTime();

        double dt = previousAxialTime == 0
                ? 0
                : (currentTime - previousAxialTime) / 1e9;

        if (dt > 0 && dt < 0.1) {
            axialIntegral += axialError * dt;
        }

        double axialDerivative = dt > 0
                ? (axialError - previousAxialError) / dt
                : 0;

        axial =
                AXIAL_P * axialError
                        + AXIAL_I * axialIntegral
                        + AXIAL_D * axialDerivative
                        + AXIAL_F;

        axial = Math.max(
                -MAX_AUTO_AXIAL,
                Math.min(MAX_AUTO_AXIAL, axial)
        );

        previousAxialError = axialError;
        previousAxialTime = currentTime;

        // Override manual yaw
        yaw = -autoYaw;

        // Alignment checks
        axialAligned =
                Math.abs(axialError) <= AXIAL_ERROR_TOL;

        yawAligned =
                Math.abs(yawError) <= YAW_ERROR_TOL;

        // Determine why the robot cannot launch.
        if (!yawAligned) {
            launchBlockReason = "YAW NOT ALIGNED";
            goodReadings = 0;

        } else if (!axialAligned) {
            launchBlockReason = "DISTANCE NOT ALIGNED";
            goodReadings = 0;

        } else if (!launcherReady) {
            launchBlockReason = "LAUNCHER NOT AT SPEED";
            goodReadings -= 1;
        } else {
            launchBlockReason = "READY TO LAUNCH";
            aimLocked = true;
            goodReadings += 1;
        }
    }


    // =========================================================
    // DRIVE
    // =========================================================

    private void drive() {

        // -------------------------
        // Manual drive input
        // -------------------------

        axial = -gamepad1.left_stick_y;
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;


        // -------------------------
        // Automatic aiming
        // Overrides manual axial/yaw
        // when right trigger is held.
        // -------------------------

        autoAim();


        // -------------------------
        // Mecanum calculations
        // -------------------------

        frontLeftPower = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;

        backLeftPower = axial - lateral + yaw;
        backRightPower = axial + lateral - yaw;


        // -------------------------
        // Normalize motor powers
        // Prevent any motor from exceeding 1.0.
        // -------------------------

        double maxPower = Math.max(
                Math.max(
                        Math.abs(frontLeftPower),
                        Math.abs(frontRightPower)
                ),
                Math.max(
                        Math.abs(backLeftPower),
                        Math.abs(backRightPower)
                )
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;

            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }


        // -------------------------
        // Send motor powers
        // -------------------------

        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);

        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);


        // -------------------------
        // Limelight servo positioning
        // -------------------------

        limelightServo.setPosition(position);
    }


    // =========================================================
    // LIMELIGHT HUD COMMUNICATION
    // =========================================================

    private void updateLimelightHUD() {

        double headingDeg =
                pinpoint.getHeading(AngleUnit.DEGREES);

        // Bit flags:
        // 1 = Auto aim active
        // 2 = Target locked
        // 4 = Robot connected

        int flags = 4;

        if (gamepad1.right_trigger >= 0.5) {
            flags |= 1;
        }

        if (aimLocked) {
            flags |= 2;
        }

        limelight.updatePythonInputs(new double[]{

                headingDeg,                              // [0] Heading
                launcher.getVelocity(),                  // [1] Launcher velocity
                vel,                                     // [2] Launcher target
                Math.toDegrees(angularVelocity),         // [3] Yaw velocity
                flags,                                   // [4] Status flags
                forwardVelocity,                         // [5] Forward velocity
                sidewaysVelocity,                        // [6] Sideways velocity
                getAlliance()                            // [7] Alliance
        });
    }


    // =========================================================
    // TELEMETRY
    // =========================================================


    private void updateTelemetry() {


        telemetry.addLine(order(0, "========== TARGETING =========="));

        telemetry.addData(
                order(1, "Target"),
                targetFound ? CLUSTERS[selectedCluster] : "NO TARGET"
        );

        telemetry.addData(
                order(2, "Launch Status"),
                launchBlockReason
        );

        telemetry.addData(
                order(3, "Aim Locked"),
                aimLocked ? "YES" : "NO"
        );

        telemetry.addData(
                order(4, "Yaw Error"),
                "%.2f / %.2f deg",
                yawError,
                YAW_ERROR_TOL
        );

        telemetry.addData(
                order(5, "Distance Error"),
                "%.2f / %.2f in",
                axialError,
                AXIAL_ERROR_TOL
        );

        telemetry.addData(
                order(6, "Measured Distance"),
                "%.2f / %.2f in",
                aimAxial,
                AXIAL_TARGET_INCHES
        );

        telemetry.addData(
                order(7, "Launcher Velocity"),
                "%.0f / %.0f RPM",
                launcher.getVelocity(),
                vel
        );

        telemetry.addData(
                order(8, "Velocity Error"),
                "%.0f RPM",
                Math.abs(launcher.getVelocity() - vel)
        );

        telemetry.addData(
                order(9, "Launcher Ready"),
                launcherReady ? "YES" : "NO"
        );

        telemetry.addData(
                order(10, "Yaw Aligned"),
                yawAligned ? "YES" : "NO"
        );

        telemetry.addData(
                order(11, "Distance Aligned"),
                axialAligned ? "YES" : "NO"
        );

        telemetry.addData(
                "Good readings",goodReadings
        );

        telemetry.update();
    }
}