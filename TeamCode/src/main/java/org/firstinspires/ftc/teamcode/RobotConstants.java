package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * One place for every hardware name and tunable number.
 * Change values here, not inside subsystems or OpModes.
 */
public final class RobotConstants {

    private RobotConstants() {} // constants only; never instantiated

    // ---- Hardware map names (must match the Driver Hub configuration) ----
    public static final String FRONT_LEFT_MOTOR  = "FL";
    public static final String BACK_LEFT_MOTOR   = "BL";
    public static final String FRONT_RIGHT_MOTOR = "FR";
    public static final String BACK_RIGHT_MOTOR  = "BR";
    public static final String PINPOINT          = "pinpoint";
    public static final String LIMELIGHT         = "limelight";
    public static final String LAUNCHER_MOTOR    = "launcher";
    public static final String FEEDER_SERVO      = "feeder";
    public static final String INTAKE_MOTOR      = "intake";

    // ---- Drive motor directions (flip one if that wheel spins the wrong way) ----
    public static final DcMotor.Direction FRONT_LEFT_DIRECTION  = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction BACK_LEFT_DIRECTION   = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction FRONT_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction BACK_RIGHT_DIRECTION  = DcMotor.Direction.FORWARD;

    // ---- Odometry (TODO: measure) ----
    public static final double PINPOINT_X_OFFSET_MM = 105; // from last year's robot - TODO re-measure on the new one
    public static final double PINPOINT_Y_OFFSET_MM = 38;  // from last year's robot - TODO re-measure on the new one

    // ---- Launcher (live-tunable values are in LauncherConfig) ----
    public static final double LAUNCHER_MAX_RPM = 6000;      // free speed of the flywheel motor
    public static final double LAUNCHER_TICKS_PER_REV = 28;  // 6000 RPM goBILDA motor, no gearbox (check the spec sheet)

    // ---- Drivetrain: 435 RPM 12V motors ----
    public static final double DRIVE_MAX_RPM = 435;
    public static final double DRIVE_TICKS_PER_REV = 383.6;  // 435 RPM goBILDA motor (check the spec sheet)

    // ---- Aiming (TODO: tune) ----
    public static final double AIM_KP = 0;
    public static final double AIM_DEADBAND_DEG = 1.0;
}
