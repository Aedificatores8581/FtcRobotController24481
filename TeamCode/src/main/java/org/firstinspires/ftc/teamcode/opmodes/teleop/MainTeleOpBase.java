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

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Shared Limelight Python-SnapScript aiming code.
 * Not an OpMode by itself: run MainTeleOpRed or MainTeleOpBlue.
 * Dashboard tuning values live here, so both OpModes share them.
 */
@Config
public abstract class MainTeleOpBase extends LinearOpMode {

    // Alliance codes sent to the SnapScript (llrobot[7])
    protected static final int ALLIANCE_RED = 1;
    protected static final int ALLIANCE_BLUE = 2;

    /** Which alliance's clusters this OpMode is allowed to target. */
    protected abstract int getAlliance();

    private boolean clusterAllowed(int cluster) {
        if (getAlliance() == ALLIANCE_RED) {
            return cluster == 0 || cluster == 1;   // Red Scoring, Red Audience
        }
        if (getAlliance() == ALLIANCE_BLUE) {
            return cluster == 2 || cluster == 3;   // Blue Audience (38-41), Blue Scoring (42-45)
        }
        return true;
    }

    private static double deadzone(double v, double dz) {
        return Math.abs(v) < dz ? 0 : v;
    }

    private static double clamp1(double v) {
        return Math.max(-1, Math.min(1, v));
    }

    private String allianceName() {
        return getAlliance() == ALLIANCE_RED ? "RED" : "BLUE";
    }

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
            new PIDFCoefficients(28, 1, 0, 13);

    public static double vel = 1920;
    public static double position = 0.15;

    // ==========================================
    // AUTOMATIC DRIVE SETTINGS
    // ==========================================
    public static double axialAdjust = 0.9;
    public static double axialForwardGain = 0.06;
    public static double maxAutoAxial = 0.7;

    public static double yawGain = 0.025;
    public static double rotationVelocityGain = 0.12;
    public static double maxAutoYaw = 0.7;

    public static double axialDeadband = 1.0;
    public static double yawDeadband = 0.7;

    public static final int SMOOTHING_WINDOW = 10;
    public static double SMOOTHING_ALPHA = 0.2;
    public static double yawOffsetDegrees = -9.0;

    // Limelight pipeline slot that holds the Python SnapScript
    public static int pythonPipeline = 3;
    // Ignore Limelight results older than this (ms)
    public static double maxStalenessMs = 150;

    // "TARGET LOCKED" when the smoothed aim is inside these windows
    // Yaw: |tx| < lockYawTolerance
    // Axial: |ty - axialAdjust| < lockAxialTolerance (axialAdjust is where axial drive stops)
    public static double lockYawTolerance = 0.7;
    public static double lockAxialTolerance = 1;

    // gamepad2 (flight stick) stick deadzone, so a resting axis can't make the robot creep
    public static double stickDeadzone = 0.05;

    // ==========================================
    // TAG ODOMETRY + HIVE TIP + PATH TO SHOT POSITION  (added)
    // Turn any of these off from the Dashboard if something misbehaves.
    // ==========================================
    public static boolean useTagOdometry = true;   // blend tag poses into the Pinpoint
    public static boolean usePathing = true;       // drive to the shot spot when no tag is seen
    public static double fixGain = 0.35;           // 1 = jump to the tag pose, 0 = ignore it
    public static double fixSnapM = 0.30;          // bigger error = needs 3 agreeing fixes to jump
    public static double fixMaxSpeedMps = 0.8;     // only blend while slower than this
    public static double fixMaxYawDps = 90;
    public static double fixMinIntervalMs = 150;
    public static double poseTrustSec = 20;        // pathing only if a fix is newer than this
    public static double maxTravelSinceFixM = 6;

    // Camera mounting (must match CAM_* in the SnapScript)
    public static double cameraPitchDeg = 25.2;
    public static double cameraHeightM = 0.30;
    public static double cameraFwdM = 0.0;
    public static double robotLengthIn = 18;

    // ty to hold on the AUDIENCE plate when it is the plate that is up.
    // -999 = derive it so the robot shoots from the same distance as for the scoring plate.
    public static double audienceAxialAdjust = -999;
    public static int tipFlipCount = 5;            // frames that must agree before a tip change is accepted
    public static boolean assumeScoringIfTipUnknown = false;

    public static double pathMaxPower = 0.6;
    public static double pathMinPower = 0.12;
    public static double pathPosGain = 1.2;        // power per meter near the goal
    public static double pathYawGain = 0.6;        // power per radian
    public static double pathMaxYaw = 0.5;
    public static double pathGoalTolM = 0.05;

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
            "Blue Audience",
            "Blue Scoring"
    };

    double aimYaw = 0;
    double aimAxial = 0;

    double targetLauncherVelocity = -1920;

    double autoYaw = 0;
    double yawError = 0;
    double axialError = 0;
    boolean targetFound = false;

    int selectedCluster = -1;

    int clusterTagCount = 0;
    int totalTags = 0;
    double targetRangeIn = 0;
    double headingDeg = 0;
    boolean aimLocked = false;

    // ==========================================
    // TAG ODOMETRY / HIVE TIP / PATHING STATE  (added)
    // ==========================================
    double odoX, odoY, odoH;                  // Pinpoint pose, field frame (m, m, rad)
    private double lastOdoX, lastOdoY;
    boolean poseCalibrated = false;
    double lastFixSec = -1e9;
    private double lastFixMs = 0;
    double travelSinceFix = 0;
    int poseTagsUsed = 0;
    private int lastFrame = -1;
    private int badFixCount = 0;
    private double badFixX, badFixY;
    /** Per hive half (0 = red, 1 = blue): 0 unknown, 1 SCORING plate up, 2 AUDIENCE plate up. */
    int[] tipState = {0, 0};
    private int[] tipCand = {0, 0};
    private int[] tipVotes = {0, 0};
    String pathStatus = "idle";
    boolean pathing = false;
    double pathDistToGoal = 0;
    double impliedPitchNow = Double.NaN;
    double impliedPitchWall = Double.NaN;

    private static final String[] TIP_TXT = {"unknown", "SCORING up", "AUDIENCE up"};

    private double nowSec() { return System.nanoTime() / 1e9; }

    private static double wrapPi(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private int myHalf() { return getAlliance() == ALLIANCE_RED ? 0 : 1; }

    /** ty the aim loop should hold for this cluster (scoring uses the original axialAdjust). */
    private double tyGoalFor(int cluster) {
        if (ShotPlanner.isScoring(cluster)) return axialAdjust;
        if (audienceAxialAdjust > -900) return audienceAxialAdjust;
        return ShotPlanner.audienceTy(cameraHeightM, cameraPitchDeg, axialAdjust);
    }

    private void readOdo() {
        odoX = pinpoint.getPosX(DistanceUnit.MM) / 1000.0;
        odoY = pinpoint.getPosY(DistanceUnit.MM) / 1000.0;
        odoH = pinpoint.getHeading(AngleUnit.RADIANS);
        travelSinceFix += Math.hypot(odoX - lastOdoX, odoY - lastOdoY);
        lastOdoX = odoX;
        lastOdoY = odoY;
    }

    private boolean poseTrusted() {
        return poseCalibrated
                && (nowSec() - lastFixSec) < poseTrustSec
                && travelSinceFix < maxTravelSinceFixM;
    }

    /** Blend a tag-solved pose into the Pinpoint. */
    private void applyTagPose(int tagsUsed, double x, double y, double h) {
        double nowMs = System.currentTimeMillis();
        if (nowMs - lastFixMs < fixMinIntervalMs) return;
        double ex = x - odoX, ey = y - odoY, eh = wrapPi(h - odoH);
        double err = Math.hypot(ex, ey);
        boolean still = Math.hypot(forwardVelocity, sidewaysVelocity) <= fixMaxSpeedMps
                && Math.abs(Math.toDegrees(angularVelocity)) <= fixMaxYawDps;
        boolean bigJump = err > fixSnapM || Math.abs(eh) > Math.toRadians(15);

        if (poseCalibrated && bigJump) {
            // far from where we think we are: believe it only when 3 fixes agree
            if (badFixCount > 0 && Math.hypot(x - badFixX, y - badFixY) < 0.10) badFixCount++;
            else badFixCount = 1;
            badFixX = x;
            badFixY = y;
            if (badFixCount < 3) return;
        }
        if (!poseCalibrated || bigJump) {
            badFixCount = 0;
        } else if (!still) {
            return;
        }

        boolean snap = !poseCalibrated || bigJump;
        double g = snap ? 1.0 : fixGain;
        double gh = snap ? 1.0 : fixGain * 0.5;
        double nx = odoX + ex * g, ny = odoY + ey * g, nh = wrapPi(odoH + eh * gh);
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, nx * 1000.0, ny * 1000.0,
                AngleUnit.RADIANS, nh));
        odoX = nx;
        odoY = ny;
        odoH = nh;
        lastOdoX = nx;
        lastOdoY = ny;
        poseCalibrated = true;
        lastFixSec = nowSec();
        lastFixMs = nowMs;
        travelSinceFix = 0;
        poseTagsUsed = tagsUsed;
    }

    /** Hold the last known tip; change it only after tipFlipCount agreeing frames. */
    private void updateTips(int red, int blue) {
        int[] rep = {red, blue};
        for (int hf = 0; hf < 2; hf++) {
            int st = rep[hf];
            if (st == 0) continue;
            if (tipState[hf] == 0 || st == tipState[hf]) {
                tipState[hf] = st;
                tipVotes[hf] = 0;
                tipCand[hf] = 0;
            } else {
                if (st == tipCand[hf]) tipVotes[hf]++;
                else { tipCand[hf] = st; tipVotes[hf] = 1; }
                if (tipVotes[hf] >= tipFlipCount) {
                    tipState[hf] = st;
                    tipVotes[hf] = 0;
                    tipCand[hf] = 0;
                }
            }
        }
    }

    /**
     * No tag target: drive (with Pinpoint) to the shooting position in front of the plate
     * that is UP on our hive half. Returns true if it set axial / lateral / yaw.
     */
    private boolean pathDrive() {
        pathing = false;
        if (!usePathing || !useTagOdometry) { pathStatus = "off"; return false; }
        if (!poseTrusted()) { pathStatus = "pose not trusted (see a tag first)"; return false; }
        int st = tipState[myHalf()];
        if (st == 0) {
            if (!assumeScoringIfTipUnknown) { pathStatus = "tip unknown (look at the hive)"; return false; }
            st = 1;
        }
        int cl = st == 1 ? ShotPlanner.scoringCluster(getAlliance())
                : ShotPlanner.audienceCluster(getAlliance());
        double ty = tyGoalFor(cl);
        double dist = ShotPlanner.shotDistance(ShotPlanner.PLATE[cl][2], cameraHeightM,
                cameraPitchDeg, ty);
        double[] goal = ShotPlanner.goal(cl, dist, cameraFwdM, robotLengthIn * 0.0254 / 2.0);
        double[][] path = ShotPlanner.path(odoX, odoY, goal[0], goal[1]);
        double[] d = ShotPlanner.drive(odoX, odoY, odoH, angularVelocity, path,
                ShotPlanner.PLATE[cl][0], ShotPlanner.PLATE[cl][1],
                pathMaxPower, pathMinPower, pathPosGain, pathYawGain, pathMaxYaw,
                rotationVelocityGain, pathGoalTolM);
        axial = d[0];
        lateral = d[1];
        yaw = d[2];
        pathDistToGoal = d[3];
        pathing = true;
        pathStatus = (d[3] < pathGoalTolM ? "AT SHOT SPOT, no tag yet: " : "PATHING to ")
                + clusters[cl] + String.format(" (%.2f m)", d[3]);
        return true;
    }

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

        // ==========================================
        // MOTOR CONFIGURATION
        // ==========================================

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

        // ==========================================
        // PINPOINT CONFIGURATION
        // ==========================================

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.update();

        // ==========================================
        // LIMELIGHT CONFIGURATION
        // ==========================================

        feeder.setPosition(0.04);

        limelight.pipelineSwitch(pythonPipeline);
        limelight.start();

        dashboard.startCameraStream(limelight, 10);

        telemetry.addLine("Limelight Aim " + allianceName() + " Ready");
        telemetry.update();

        waitForStart();

        // ==========================================
        // MAIN LOOP
        // ==========================================

        while (opModeIsActive()) {

            targetLauncherVelocity = vel;
            autoYaw = 0;
            yawError = 0;
            axialError = 0;

            // Shared "buttons" for both pads:
            //   gamepad1 left trigger  / gamepad2 dpad_up -> feeder
            //   gamepad1 right trigger / gamepad2 a       -> auto aim
            //   gamepad1 right bumper  / gamepad2 b       -> intake toggle
            boolean feedHeld = gamepad1.left_trigger >= 0.3 || gamepad2.dpad_up;
            boolean autoAimHeld = gamepad1.right_trigger >= 0.5 || gamepad2.a;

            // ==========================================
            // LAUNCHER PIDF
            // ==========================================

            launcher.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER, Pidf
            );

            // ==========================================
            // FEEDER
            // ==========================================

            if (feedHeld) {
                feeder.setPosition(0.04);
            } else {
                feeder.setPosition(0.3);
            }

            // gamepad2.dpad_up is the feed button, so only gamepad1 gets the
            // dpad_up nudge; dpad_down nudges on both pads.
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                feeder.setPosition(
                        Math.max(feeder.getPosition() - 0.01, 0)
                );
            }

            if (gamepad1.dpad_up) {
                feeder.setPosition(
                        Math.min(feeder.getPosition() + 0.01, 1)
                );
            }

            // ==========================================
            // INTAKE
            // ==========================================

            // Read both so each pad's edge flag is consumed every loop
            boolean intakeToggle1 = gamepad1.rightBumperWasPressed();
            boolean intakeToggle2 = gamepad2.bWasPressed();
            if (intakeToggle1 || intakeToggle2) {
                intakeActive = !intakeActive;
            }

            intake.setPower(intakeActive ? autoAimHeld ? 0.35 : 0.8 : 0);

            // ==========================================
            // PINPOINT VELOCITY
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

            readOdo();

            // ==========================================
            // MANUAL DRIVE
            // ==========================================

            // Both pads can drive; gamepad2 gets a deadzone.
            axial = clamp1(-gamepad1.left_stick_y
                    - deadzone(gamepad2.left_stick_y, stickDeadzone));
            lateral = clamp1(gamepad1.left_stick_x
                    + deadzone(gamepad2.left_stick_x, stickDeadzone));
            yaw = clamp1(gamepad1.right_stick_x
                    + deadzone(gamepad2.right_stick_x, stickDeadzone));

            targetFound = false;
            selectedCluster = -1;
            aimLocked = false;

            // ==========================================
            // LIMELIGHT PYTHON SNAPSCRIPT OUTPUT
            // [valid, tx, ty, cluster, tagCount, rangeM, totalTags, frame]
            // ==========================================

            LLResult result = limelight.getLatestResult();

            double[] py = null;
            boolean fresh = false;

            clusterTagCount = 0;
            totalTags = 0;
            targetRangeIn = 0;

            if (result != null) {
                py = result.getPythonOutput();
                fresh = result.getStaleness() < maxStalenessMs;
            }

            // py[6] = totalTags + 32 * (xCm + 512 * yCm)           (see SnapScript _pack)
            // py[7] = frame%100 + 100 * (heading*10 + 3600 * (tipRed + 3*tipBlue + 9*poseTags))
            int poseFlag = 0, tipRed = 0, tipBlue = 0, frameNo = -1;
            double tagX = 0, tagY = 0, tagH = 0;
            if (py != null && py.length >= 8) {
                long v6 = (long) py[6], v7 = (long) py[7];
                totalTags = (int) (v6 % 32);
                long pos = v6 / 32;
                tagX = ((int) (pos % 512) - 200) / 100.0;
                tagY = ((int) (pos / 512) - 200) / 100.0;
                frameNo = (int) (v7 % 100);
                long r7 = v7 / 100;
                tagH = Math.toRadians((int) (r7 % 3600) / 10.0);
                r7 /= 3600;
                tipRed = (int) (r7 % 3);
                r7 /= 3;
                tipBlue = (int) (r7 % 3);
                poseFlag = (int) (r7 / 3);
            } else if (py != null && py.length >= 7) {
                totalTags = (int) py[6];
            }

            // one update per camera frame
            if (fresh && frameNo >= 0 && frameNo != lastFrame) {
                lastFrame = frameNo;
                updateTips(tipRed, tipBlue);
                if (useTagOdometry && poseFlag > 0) applyTagPose(poseFlag, tagX, tagY, tagH);
            }

            if (fresh && py != null && py.length >= 6 && py[0] >= 0.5) {

                int cluster = (int) Math.round(py[3]);

                // Double-check the alliance here too, in case the
                // SnapScript on the Limelight is an older version
                if (cluster >= 0 && cluster < clusters.length
                        && clusterAllowed(cluster)) {
                    selectedCluster = cluster;

                    // Script reconstructs the cluster CENTER from the known
                    // tag layout, so this is valid even with 1 tag visible
                    aimYaw = py[1];
                    aimAxial = py[2];

                    clusterTagCount = (int) py[4];
                    targetRangeIn = py[5] * 39.37;
                }
            }

            if (selectedCluster != -1) {

                double tyGoal = tyGoalFor(selectedCluster);

                //Fix yaw mistakes

                targetFound = true;

                // ==========================================
                // SMOOTHING
                // ==========================================


                // Reset history if the selected cluster changes
                if (selectedCluster != previousSmoothingCluster) {

                    smoothingIndex = 0;
                    smoothingCount = 0;

                    previousSmoothingCluster = selectedCluster;
                }


                // Store newest readings
                aimYawHistory[smoothingIndex] = aimYaw;
                aimAxialHistory[smoothingIndex] = aimAxial;

// Advance circular buffer
                smoothingIndex = (smoothingIndex + 1) % SMOOTHING_WINDOW;

                if (smoothingCount < SMOOTHING_WINDOW) {
                    smoothingCount++;
                }

                // Calculate exponentially weighted averages
                double yawSum = 0;
                double axialSum = 0;
                double totalWeight = 0;

// Find newest reading
                int newestIndex =
                        (smoothingIndex - 1 + SMOOTHING_WINDOW)
                                % SMOOTHING_WINDOW;

                for (int age = 0; age < smoothingCount; age++) {

                    // Find the actual buffer position for this reading
                    int index =
                            (newestIndex - age + SMOOTHING_WINDOW)
                                    % SMOOTHING_WINDOW;

                    // Newer readings receive greater weight
                    double weight =
                            Math.pow(1.0 - SMOOTHING_ALPHA, age);

                    yawSum += aimYawHistory[index] * weight;
                    axialSum += aimAxialHistory[index] * weight;

                    totalWeight += weight;
                }

                smoothedAimYaw = yawSum / totalWeight;
                smoothedAimAxial = axialSum / totalWeight;

                // ==========================================
                // LOCK CHECK (drives the TARGET LOCKED HUD banner)
                // ==========================================

                aimLocked =
                        Math.abs(smoothedAimYaw) < lockYawTolerance &&
                                Math.abs(smoothedAimAxial - tyGoal) < lockAxialTolerance;


                // ==========================================
                // AUTOMATIC AIMING
                // ==========================================

                if (autoAimHeld) {

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

                    axial = (-axialError + tyGoal) * axialForwardGain;

                    axial = Math.max(-maxAutoAxial, Math.min(maxAutoAxial, axial)
                    );

                    yaw = -autoYaw;

                    if ((Math.abs(axial) <= 0.05) && (Math.abs(yaw) <= 0.05)) {
                        intakeActive = true;
                        feeder.setPosition(0.04);
                    }
                }
            } else {
                // Reset smoothing when no target is found
                smoothingIndex = 0;
                smoothingCount = 0;

                previousSmoothingCluster = -1;

                smoothedAimYaw = 0;
                smoothedAimAxial = 0;
            }

            // ==========================================
            // STOP IF AUTOMATIC AIMING HAS NO TARGET
            // ==========================================

            pathing = false;
            if (!autoAimHeld) pathStatus = "idle";
            if (autoAimHeld && !targetFound) {
                // no tag target: drive to the shot spot by odometry if we can, else stop
                if (!pathDrive()) {
                    axial = 0;
                    lateral = 0;
                    yaw = 0;
                }
            }

            // ==========================================
            // LAUNCHER
            // ==========================================

            targetLauncherVelocity = vel;
            launcher.setVelocity(targetLauncherVelocity);

            // ==========================================
            // DRIVE CALCULATIONS
            // ==========================================

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

            // ==========================================
            // SEND DRIVE POWERS
            // ==========================================

            FL.setPower(frontLeftPower);
            FR.setPower(frontRightPower);

            BL.setPower(backLeftPower);
            BR.setPower(backRightPower);

            // ==========================================
            // SEND DATA TO SNAPSCRIPT HUD
            // [heading, launcherVel, launcherTarget, yawRateDeg, flags,
            //  forwardVel, rightVel, alliance]
            //  flags: bit0 autoAim, bit1 aimLocked, bit2 robotConnected
            //  forwardVel / rightVel are ROBOT-relative m/s (Pinpoint reports
            //  field-relative, so it is rotated by the heading here)
            // ==========================================

            headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

            double hRad = Math.toRadians(headingDeg);
            double robotForwardVel = forwardVelocity * Math.cos(hRad)
                    + sidewaysVelocity * Math.sin(hRad);
            double robotLeftVel = -forwardVelocity * Math.sin(hRad)
                    + sidewaysVelocity * Math.cos(hRad);

            int flags = 0;
            if (autoAimHeld) flags |= 1;
            if (aimLocked) flags |= 2;
            flags |= 4;

            limelight.updatePythonInputs(new double[]{
                    headingDeg,
                    launcher.getVelocity(),
                    targetLauncherVelocity,
                    Math.toDegrees(angularVelocity),
                    flags,
                    robotForwardVel,
                    -robotLeftVel,
                    getAlliance()
            });

            // ==========================================
            // COLLAPSED TELEMETRY
            // ==========================================

            // Limelight / SnapScript

            telemetry.addData(
                    "Limelight",
                    "Result: %s | Fresh: %s | Tags: %d | Cluster tags: %d",
                    result != null,
                    fresh,
                    totalTags,
                    clusterTagCount
            );

            telemetry.addData(
                    "Range",
                    "%.1f in",
                    targetRangeIn
            );

            // Selected cluster

            telemetry.addData(
                    "Alliance",
                    allianceName() + " only"
            );

            telemetry.addData(
                    "Selected Cluster",
                    selectedCluster == -1
                            ? "None"
                            : clusters[selectedCluster]
            );

            telemetry.addData(
                    "Target",
                    "Found: %s | LOCKED: %s",
                    targetFound,
                    aimLocked
            );

            // Aiming values

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

            // Tag odometry / hive tip / pathing (added)
            if (selectedCluster != -1) {
                double camX = odoX + cameraFwdM * Math.cos(odoH);
                double camY = odoY + cameraFwdM * Math.sin(odoH);
                double horiz = Math.hypot(ShotPlanner.PLATE[selectedCluster][0] - camX,
                        ShotPlanner.PLATE[selectedCluster][1] - camY);
                impliedPitchNow = poseTrusted()
                        ? ShotPlanner.impliedPitch(selectedCluster, cameraHeightM, horiz, smoothedAimAxial)
                        : Double.NaN;
                // parked against our own wall, square to and in line with the hive (y = 0)
                double side = ShotPlanner.PLATE[selectedCluster][3];
                double cx = side * (ShotPlanner.FIELD_HALF - robotLengthIn * 0.0254 / 2.0) + cameraFwdM * (-side);
                double hz = Math.hypot(ShotPlanner.PLATE[selectedCluster][0] - cx,
                        ShotPlanner.PLATE[selectedCluster][1]);
                impliedPitchWall = ShotPlanner.impliedPitch(selectedCluster, cameraHeightM, hz, smoothedAimAxial);
            } else {
                impliedPitchNow = Double.NaN;
                impliedPitchWall = Double.NaN;
            }

            telemetry.addData(
                    "Tag Pose",
                    "Calibrated: %s | Trusted: %s | Last fix: %.1f s | Tags: %d",
                    poseCalibrated,
                    poseTrusted(),
                    Math.min(999, nowSec() - lastFixSec),
                    poseTagsUsed
            );

            telemetry.addData(
                    "Odometry (field m, deg)",
                    "X: %.2f | Y: %.2f | H: %.1f",
                    odoX,
                    odoY,
                    Math.toDegrees(odoH)
            );

            telemetry.addData(
                    "Hive Tip",
                    "Red half: %s | Blue half: %s | Mine: %s",
                    TIP_TXT[tipState[0]],
                    TIP_TXT[tipState[1]],
                    TIP_TXT[tipState[myHalf()]]
            );

            telemetry.addData("Path", pathStatus);

            telemetry.addData(
                    "Implied pitch",
                    "From odometry: %.1f | If parked at wall in line: %.1f | Set: %.1f",
                    impliedPitchNow,
                    impliedPitchWall,
                    cameraPitchDeg
            );

            // Automatic drive

            telemetry.addData(
                    "Auto Drive",
                    "Yaw: %.3f | Axial: %.3f",
                    autoYaw,
                    axial
            );

            // Odometry

            telemetry.addData(
                    "Pinpoint Velocity",
                    "Forward: %.3f | Sideways: %.3f | Rotation: %.3f",
                    forwardVelocity,
                    sidewaysVelocity,
                    angularVelocity
            );

            // Launcher

            telemetry.addData(
                    "Launcher",
                    "Target: %.0f | Actual: %.0f | Current: %.0f mA",
                    targetLauncherVelocity,
                    launcher.getVelocity(),
                    launcher.getCurrent(CurrentUnit.MILLIAMPS)
            );

            // Mechanisms

            telemetry.addData(
                    "Mechanisms",
                    "Intake: %s | Feeder: %.2f",
                    intakeActive,
                    feeder.getPosition()
            );

            // Drive motor powers

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
}