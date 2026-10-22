package org.firstinspires.ftc.teamcode;

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

    // ---- Odometry (TODO: measure) ----
    public static final double PINPOINT_X_OFFSET_MM = 0;
    public static final double PINPOINT_Y_OFFSET_MM = 0;

    // ---- Launcher (TODO: tune) ----
    public static final double LAUNCHER_RPM_TOLERANCE = 50;

    // ---- Aiming (TODO: tune) ----
    public static final double AIM_KP = 0;
    public static final double AIM_DEADBAND_DEG = 1.0;
}
