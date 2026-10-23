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
    public static double KP = 100.0; // KP/KI/KD/KF start from last year's working launcher (different motor, re-tune)
    public static double KI = 0.1;
    public static double KD = 25.0;
    public static double KF = 11.7; // = 32767 / max ticks per sec (6000 RPM * 28 / 60 = 2800)

    public static double RPM_TOLERANCE = 50; // "ready" when within this many RPM
    public static double DEFAULT_RPM = 3000;

    public static double FEEDER_REST_POS = 0.0; // TODO: find on the robot
    public static double FEEDER_FEED_POS = 0.5; // TODO: find on the robot
}
