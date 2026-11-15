package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.AimConfig;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.AimController;
import org.firstinspires.ftc.teamcode.util.Alliance;

/*
 * Shared code for the two main TeleOps (BlueAimingTeleOp, RedAimingTeleOp).
 * Same driving and aiming logic as the old StandaloneBlue/RedAimingTeleop, now on the Robot subsystems.
 * The only difference between the two is which alliance's tags it aims at.
 *
 * Left stick: drive/strafe. Right stick: turn.
 * Left trigger: feed (hold). D-pad up/down: nudge the feeder servo. Right bumper: toggle intake.
 * Right trigger (hold, 0.5+): auto-aim at this alliance's closest tag cluster (strafe stays on the stick).
 *   Once lined up, intake and feeder start.
 *   With no tag in view the robot stops (the sticks are ignored while the trigger is held).
 * The launcher spins continuously at LauncherConfig.TARGET_VELOCITY.
 */
public abstract class AimingTeleOpBase extends OpMode {

    private static final double AIM_TRIGGER = 0.5;
    private static final double FEED_TRIGGER = 0.3;
    private static final double FEEDER_NUDGE = 0.01;

    private Robot robot;
    private final AimController aim = new AimController();

    /** Which alliance's tags to aim at. */
    protected abstract Alliance alliance();

    @Override
    public void init() {
        RobotLog.i("AimingTeleOp: Initializing robot");
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        robot.vision.setAimAlliance(alliance());
        robot.launcher.coastWhenStopped();
        robot.launcher.feed(); // the standalone started with the feeder at its feed position
    }

    @Override
    public void loop() {
        // Read Pinpoint + Limelight first so this loop's aiming uses fresh readings (as the standalone did).
        robot.update();

        robot.launcher.spinUp();

        // Feeder
        if (gamepad1.left_trigger >= FEED_TRIGGER) {
            robot.launcher.feed();
        } else {
            robot.launcher.retractFeeder();
        }
        if (gamepad1.dpad_down) {
            robot.launcher.nudgeFeeder(-FEEDER_NUDGE);
        }
        if (gamepad1.dpad_up) {
            robot.launcher.nudgeFeeder(FEEDER_NUDGE);
        }

        // Intake
        if (gamepad1.rightBumperWasPressed()) {
            robot.intake.toggle();
        }

        // Manual drive
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Auto-aim
        boolean aiming = gamepad1.right_trigger >= AIM_TRIGGER;
        boolean targetFound = robot.vision.hasAimTarget();

        if (aiming && targetFound) {
            aim.calculate(robot.vision.getAimTx(), robot.vision.getAimTy(),
                    robot.odo.getHeadingVelocityRadPerSec());
            axial = aim.forward();
            yaw = aim.turn();

            if (aim.isAimed()) {
                robot.intake.on();
                robot.launcher.feed();
            }
        }

        if (aiming && !targetFound) {
            axial = 0;
            lateral = 0;
            yaw = 0;
        }

        robot.vision.setServoPosition(AimConfig.LIMELIGHT_SERVO_POS); // every loop so Dashboard changes apply

        // Apply the drive now (not next robot.update()) so it matches this loop's aiming.
        robot.drive.drive(axial, lateral, yaw);
        robot.drive.update();

        telemetry.addData("Alliance", alliance());
        telemetry.addData("Target found", targetFound);
        telemetry.addData("Auto drive", "axial %.3f  yaw %.3f", axial, yaw);
    }

    @Override
    public void stop() {
        RobotLog.i("AimingTeleOp: Stopping robot");
        robot.stop();
    }
}
