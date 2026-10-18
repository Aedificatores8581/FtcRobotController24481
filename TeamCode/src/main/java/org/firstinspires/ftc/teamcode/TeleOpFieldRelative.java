package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Field-relative driving: push the stick toward the far wall and the robot goes toward the
 * far wall, no matter which way the robot is facing. (TeleOpBasic is "robot-relative":
 * forward means the robot's own front.)
 *
 *     Left stick     = which direction to travel across the field
 *     Right stick    = turn
 *     A button       = tell the robot "the way you're facing right now is field-forward"
 *     Left bumper    = (hold) drive robot-relative instead
 *
 * It works by reading the robot's heading from the Control Hub's built-in gyro (the IMU)
 * and rotating the stick direction by the opposite angle.
 *
 * Needs an IMU in the robot configuration, and the hub mounting set in RobotConstants.java.
 * Based on the SDK sample RobotTeleopMecanumFieldRelativeDrive.
 */
@TeleOp(name = "TeleOp Field Relative", group = "TeleOp")
public class TeleOpFieldRelative extends LinearOpMode {

    private static final String TAG = "TeleOpFieldRelative";

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private IMU imu;

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addLine("Point the robot away from you, then press START.");
        telemetry.update();

        waitForStart();
        imu.resetYaw(); // whatever direction we face now counts as "field forward"

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                imu.resetYaw();
                RobotLog.ii(TAG, "Heading reset");
            }

            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double turn    =  gamepad1.right_stick_x;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (gamepad1.left_bumper) {
                drive(forward, strafe, turn); // robot-relative
            } else {
                // Rotate the stick direction by minus the robot's heading.
                double cos = Math.cos(-heading);
                double sin = Math.sin(-heading);
                double fieldForward = forward * cos - strafe * sin;
                double fieldStrafe  = forward * sin + strafe * cos;
                drive(fieldForward, fieldStrafe, turn);
            }

            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addLine("A = reset heading, hold LB = robot-relative");
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

        imu = hardwareMap.get(IMU.class, RobotConstants.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotConstants.HUB_LOGO_DIRECTION, RobotConstants.HUB_USB_DIRECTION)));
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
