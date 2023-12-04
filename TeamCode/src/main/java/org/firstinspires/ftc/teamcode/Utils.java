package org.firstinspires.ftc.teamcode;

public class Utils {
    public static double degreesToPos(double number) {
        return number / 180;
    }
    public static double accel(double power, double denominator) {
        return power / denominator;
    }
}
