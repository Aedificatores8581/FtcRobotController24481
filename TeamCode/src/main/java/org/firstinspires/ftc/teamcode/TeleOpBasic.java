package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * The simplest driver-controlled mode: mecanum drive with the gamepad.
 *
 *     Left stick   = drive forward/backward and strafe sideways
 *     Right stick  = turn
 *
 * Start here. TeleOpSpeedToggle and TeleOpFieldRelative build on this one.
 * Names, directions, and power levels live in RobotConstants.java.
 */
@TeleOp(name = "TeleOp Basic", group = "TeleOp")
public class TeleOpBasic extends LinearOpMode {

    private static final String TAG = "TeleOpBasic";

    private DcMotor frontLeft, backLeft, frontRight, backRight;

    @Override
    public void runOpMode() {
        // ---- INIT ----
        initHardware();

        // If SimpleAuto (or BlackboardAuto) ran first, it may have left notes for us.
        // See BlackboardAuto.java. getOrDefault gives "unknown" if nothing was saved.
        telemetry.addData("Alliance from auto", blackboard.getOrDefault(BlackboardAuto.ALLIANCE_KEY, "unknown"));
        telemetry.addData("Status", "Ready. Press START.");
        telemetry.update();

        waitForStart();
        RobotLog.ii(TAG, "START pressed");

        // ---- DRIVE LOOP: repeats until STOP ----
        while (opModeIsActive()) {
            // Pushing a stick forward gives a NEGATIVE number, so flip the sign for "forward".
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double turn    =  gamepad1.right_stick_x;

            drive(forward, strafe, turn);

            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
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

    /**
     * Mix the three driver inputs into four wheel powers.
     * Each input is -1.0 to 1.0. Positive turn = clockwise.
     */
    private void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // If any wheel would go past 100%, scale all four down together so the robot
        // still moves in the direction the driver asked for.
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                            Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }
}
