package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/*
 * Launcher values you can change LIVE from FTC Dashboard (http://192.168.43.1:8080/dash).
 * They must stay 'public static' and NOT 'final' for Dashboard to edit them.
 * Once a value feels right, write it down here so it survives a restart.
 *
 * Tuning order: kF first, then kP, then kD/kI only if needed.
 */
@Config
public class LauncherConfig {
    // Tuned and tested on the robot (SuperTestTeleop bench test).
    public static double KP = 120.0;
    public static double KI = 0.67;
    public static double KD = 12.0;
    public static double KF = 150.0;

    // Flywheel speed is in encoder ticks per second everywhere (no RPM conversion).
    public static double TARGET_VEL = 950;          // ticks/sec, the speed we shoot at
    public static double VEL_TOLERANCE = 25;        // "ready" when within this many ticks/sec
    public static double READY_HOLD_SECONDS = 0.1;  // must hold speed this long before a shot

    // Auto shooting, starting guesses. Tune live from Dashboard.
    public static int AUTO_SHOT_COUNT = 3;                  // pieces to shoot at the hive in auto
    public static double SHOOT_READY_TIMEOUT_SECONDS = 2.0; // fire anyway after this long
    public static double SHOOT_FEED_HOLD_SECONDS = 0.3;     // how long to hold the feeder forward
    public static double SHOOT_FEED_PAUSE_SECONDS = 0.3;    // wait before the next shot

    public static double FEEDER_REST_POS = 0.3; // held while not feeding
    public static double FEEDER_FEED_POS = 0.04; // held while feeding
}
