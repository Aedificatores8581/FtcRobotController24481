package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * Same driving as TeleOpBasic, plus a SLOW MODE button. This shows "edge detection".
 *
 *     Left stick   = drive forward/backward and strafe sideways
 *     Right stick  = turn
 *     A button     = switch slow mode on/off
 *
 * THE PROBLEM: the loop runs hundreds of times a second. If we wrote
 *     if (gamepad1.a) { slow = !slow; }
 * one quick tap is "true" for dozens of loop passes, so slow mode flips back and forth
 * and ends up on a random setting.
 *
 * THE FIX: gamepad1.aWasPressed() is true exactly ONCE per press, no matter how long
 * the button is held. (gamepad1.aWasReleased() is true once when you let go.)
 * Every button has the same pair, e.g. bWasPressed(), leftBumperWasPressed().
 *
 * Slow-mode strength lives in RobotConstants.SLOW_MODE_SCALE.
 */
@TeleOp(name = "TeleOp Speed Toggle", group = "TeleOp")
public class TeleOpSpeedToggle extends LinearOpMode {

    private static final String TAG = "TeleOpSpeedToggle";

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private boolean slowMode = false;

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData("Status", "Ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Runs once per press of A, not once per loop pass.
            if (gamepad1.aWasPressed()) {
                slowMode = !slowMode;
                RobotLog.ii(TAG, "Slow mode %s", slowMode ? "ON" : "OFF");
            }

            double scale = slowMode ? RobotConstants.SLOW_MODE_SCALE : 1.0;
            drive(-gamepad1.left_stick_y * scale,
                   gamepad1.left_stick_x * scale,
                   gamepad1.right_stick_x * scale);

            telemetry.addData("Slow mode", slowMode ? "ON" : "off");
            telemetry.addData("A held down?", gamepad1.a); // compare: this stays true while held
            telemetry.update();
        }
    }

    private void initHardware() {
        frontLeft  = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_LEFT_MOTOR);
        backLeft   = hardwareMap.get(DcMotor.class, RobotConstants.BACK_LEFT_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_RIGHT_MOTOR);
        backRight  = hardwareMap.get(DcMotor.class, RobotConstants.BACK_RIGHT_MOTOR);

        frontLeft.setDirection(RobotConstants.FRONT_LEFT_DIRECTION);
        backLeft.setDirection(RobotConstants.BACK_LEFT_DIRECTION);
        frontRight.setDirection(RobotConstants.FRONT_RIGHT_DIRECTION);
        backRight.setDirection(RobotConstants.BACK_RIGHT_DIRECTION);

        for (DcMotor m : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                            Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }
}
