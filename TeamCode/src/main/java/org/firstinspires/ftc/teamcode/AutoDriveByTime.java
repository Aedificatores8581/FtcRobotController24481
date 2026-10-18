package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * The simplest possible autonomous: turn the motors on, wait a set number of seconds, stop.
 * No sensors needed.
 *
 * The catch: a seconds-based move depends on battery voltage, floor friction, and wheel slip.
 * The same "2 seconds" goes a different distance on a fresh battery than a tired one.
 * Compare with SimpleAuto, which measures actual distance with the Pinpoint odometry computer.
 *
 * Try it: run it 3 times and measure how far the robot goes each time.
 *
 * The routine is the list of calls after waitForStart(). Edit that list to change the path.
 * Names, directions, and power levels live in RobotConstants.java.
 */
@Autonomous(name = "Auto Drive By Time", group = "Auto")
public class AutoDriveByTime extends LinearOpMode {

    private static final String TAG = "AutoDriveByTime";

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData("Status", "Ready. Press START.");
        telemetry.update();

        waitForStart();

        // ---- ROUTINE: edit this list. Each call is (seconds). Negative power = backward. ----
        driveForTime(1.5, RobotConstants.DRIVE_POWER);   // forward 1.5 s
        turnForTime(0.6, RobotConstants.TURN_POWER);     // turn left 0.6 s
        driveForTime(1.0, -RobotConstants.DRIVE_POWER);  // backward 1.0 s

        telemetry.addData("Status", "Done");
        telemetry.update();
        RobotLog.ii(TAG, "Routine finished");
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

    /** Drive straight for a number of seconds (negative power = backward). */
    private void driveForTime(double seconds, double power) {
        RobotLog.ii(TAG, "driveForTime(%.1f s, power %.2f)", seconds, power);
        setPowers(power, power, power, power);
        waitSeconds(seconds, "Drive");
        stopMotors();
    }

    /** Turn in place to the left for a number of seconds (negative power = right). */
    private void turnForTime(double seconds, double power) {
        RobotLog.ii(TAG, "turnForTime(%.1f s, power %.2f)", seconds, power);
        setPowers(-power, -power, power, power);
        waitSeconds(seconds, "Turn");
        stopMotors();
    }

    private void waitSeconds(double seconds, String label) {
        timer.reset();
        // opModeIsActive() turns false when STOP is pressed, so the robot stops right away.
        while (opModeIsActive() && timer.seconds() < seconds) {
            telemetry.addData(label, "%.1f / %.1f s", timer.seconds(), seconds);
            telemetry.update();
        }
    }

    private void setPowers(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    private void stopMotors() {
        setPowers(0, 0, 0, 0);
    }
}
