package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

public class PIDFControllerUtility {
    private double p;
    private double i;
    private double d;
    private double f;
    private double ticksToDeg;
    private PIDController controller;

    public PIDFControllerUtility(double p, double i, double d, double f, double ticksToDeg) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.ticksToDeg = ticksToDeg;
        controller = new PIDController(p, i, d);
    }

    public double calculate(double ref, double target) {
        controller.setPID(p, i, d);
        double pid = controller.calculate(ref, target);
        double ff = Math.cos(Math.toRadians(target / ticksToDeg)) * f;
        return pid + ff;
    }
}
