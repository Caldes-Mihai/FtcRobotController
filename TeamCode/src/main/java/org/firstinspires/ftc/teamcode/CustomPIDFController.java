package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

public class CustomPIDFController extends PIDController {
    private double p;
    private double i;
    private double d;
    private double f;
    private double ticksToDeg;

    public CustomPIDFController(double p, double i, double d, double f, double rpm) {
        super(p, i, d);
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.ticksToDeg = rpm / 180;
    }

    @Override
    public double calculate(double ref, double target) {
        this.setPID(p, i, d);
        double pid = this.calculate(ref, target);
        double ff = Math.cos(Math.toRadians(target / ticksToDeg)) * f;
        return pid + ff;
    }
}
