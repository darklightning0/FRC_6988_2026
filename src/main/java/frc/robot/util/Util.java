package frc.robot.util;

public final class Util {
    public static double sign(double value) { return (value >= 0) ? 1 : -1; }
    
    // value: 0 ~ 1
    // e.g.: lerp(0, a, b) => a ___ lerp(1, a, b) => b
    public static double lerp(double value, double lower, double upper) {
        return lower + (upper - lower) * value;
    }

    public static double signedLerp(double value, double lower, double upper) {
        return sign(value) * lerp(Math.abs(value), lower, upper);
    }
    
    // value: 0 ~ 1
    // e.g.: lerp(0, a, b) => a ___ lerp(1, a, b) => b
    public static double map(double lower, double upper, double x, double newLower, double newUpper) {
        double val = (x - lower) / (upper - lower);
        return lerp(val, newLower, newUpper);
    }

    public static double clamp(double value, double lower, double upper) {
        return Math.max(Math.min(Math.abs(value), 1), 0);
    }

    public static double deadband(double value, double epsilon) {
        if (Math.abs(value) < epsilon) return 0;
        else return value;
    }
}
