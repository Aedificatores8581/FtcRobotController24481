package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Shows a value on telemetry and writes it to the robot log. Use for events, not every loop.
public final class PedroLog {

    private static final String TAG = "Pedro";

    // private constructor to prevent instantiation
    private PedroLog() {}

    // logs a message with a key to both telemetry and the robot log
    public static void log(Telemetry telemetry, String key, String message) {
        telemetry.addData(key, message);
        RobotLog.ii(TAG, key + ": " + message);
    }

    // formats a Pose object into a readable string for telemetry and logging
    public static String format(Pose pose) {
        return String.format("x %.1f  y %.1f  heading %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
