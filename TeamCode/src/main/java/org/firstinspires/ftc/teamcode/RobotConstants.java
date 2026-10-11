package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * One place for every name and number the robot code depends on.
 * Change values here, not inside the OpModes.
 *
 * Usage from any other file:  RobotConstants.DRIVE_POWER
 */
public final class RobotConstants {

    private RobotConstants() {} // holds constants only; never created as an object

    // ---- Hardware names. Must match the robot configuration on the Driver Hub exactly. ----
    public static final String FRONT_LEFT_MOTOR  = "FL";
    public static final String BACK_LEFT_MOTOR   = "BL";
    public static final String FRONT_RIGHT_MOTOR = "FR";
    public static final String BACK_RIGHT_MOTOR  = "BR";
    public static final String PINPOINT          = "pinpoint";

    // ---- Motor directions. Flip one if that wheel spins the wrong way when driving forward. ----
    public static final DcMotor.Direction FRONT_LEFT_DIRECTION  = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction BACK_LEFT_DIRECTION   = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction FRONT_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction BACK_RIGHT_DIRECTION  = DcMotor.Direction.FORWARD;

    // ---- Pinpoint odometry ----
    // Pod offsets from the robot's center of rotation, in millimeters.
    //   X pod offset: how far LEFT of center the forward-facing pod is (negative = right of center).
    //   Y pod offset: how far FORWARD of center the sideways pod is   (negative = behind center).
    // TODO: measure these on our robot. The values below are goBILDA's demo chassis, not ours.
    public static final double X_POD_OFFSET_MM = -84.0;
    public static final double Y_POD_OFFSET_MM = -168.0;

    // Which goBILDA pod is on the robot: goBILDA_4_BAR_POD or goBILDA_SWINGARM_POD.
    public static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    // Flip to REVERSED if X doesn't go up when pushed forward / Y doesn't go up when pushed left.
    public static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // ---- Autonomous motor power (0.0 to 1.0). Start low; raise once moves are reliable. ----
    public static final double DRIVE_POWER  = 0.3;
    public static final double STRAFE_POWER = 0.3;
    public static final double TURN_POWER   = 0.25;
}
