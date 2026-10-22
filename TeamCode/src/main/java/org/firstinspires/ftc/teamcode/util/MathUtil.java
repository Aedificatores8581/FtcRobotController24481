package org.firstinspires.ftc.teamcode.util;

public final class MathUtil {

    private MathUtil() {}

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double deadband(double value, double threshold) {
        return Math.abs(value) < threshold ? 0 : value;
    }

    /** Wraps an angle in degrees to [-180, 180). */
    public static double angleWrap(double degrees) {
        // TODO
        return degrees;
    }
}
