package org.firstinspires.ftc.teamcode;

public class Utils {
    public static double degreesToPos(double number) {
        return number / 180;
    }
    public static double accel(double power, double denominator) {
        return Math.signum(power) * Math.pow(Math.abs(power / denominator), 5D / 3) * 0.9;
    }
}
