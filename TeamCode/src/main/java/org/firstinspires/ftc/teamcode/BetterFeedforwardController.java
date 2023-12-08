package org.firstinspires.ftc.teamcode;

import java.util.function.DoubleSupplier;

public class BetterFeedforwardController {
    public DoubleSupplier kV = () -> 0, kA = () -> 0, kS = () -> 0, kG = () -> 0, kCos = () -> 0;
    public double maxOutput = 1.0;
    public double outputMulti = 1.0;

    public double kSTolerance = 1e-6;

    public BetterFeedforwardController() {}

    public void setFeedforwardCoefficients(DoubleSupplier kV, DoubleSupplier kA, DoubleSupplier kS, DoubleSupplier kG, DoubleSupplier kCos)
    {
        setFeedforwardCoefficients(kV, kA, kS, kG);
        this.kCos = kCos;
    }

    public void setFeedforwardCoefficients(DoubleSupplier kV, DoubleSupplier kA, DoubleSupplier kS, DoubleSupplier kG)
    {
        setFeedforwardCoefficients(kV, kA, kS);
        this.kG = kG;
    }

    public void setFeedforwardCoefficients(DoubleSupplier kV, DoubleSupplier kA, DoubleSupplier kS)
    {
        setFeedforwardCoefficients(kV, kA);
        this.kS = kS;
    }

    public void setFeedforwardCoefficients(DoubleSupplier kV, DoubleSupplier kA)
    {
        this.kV = kV;
        this.kA = kA;
    }

    public double calculate(double targetVelocity, double targetAcceleration, double kCosAngle)
    {
        double output;
        if (Double.isNaN(targetVelocity)) targetVelocity = 0;
        if (Double.isNaN(targetAcceleration)) targetAcceleration = 0;
        output = kV.getAsDouble() * targetVelocity + kA.getAsDouble() * targetAcceleration + kG.getAsDouble();
        if (Math.abs(targetVelocity) > kSTolerance) {
            output += kS.getAsDouble() * Math.signum(targetVelocity);
        }
        if (!Double.isNaN(kCosAngle)) {
            output += kCos.getAsDouble() * Math.cos(kCosAngle);
        }
        output *= outputMulti;
        if (Double.isNaN(output)) output = 0.0;
        output = Math.max(-maxOutput, Math.min(maxOutput, output));
        return output;
    }

    public double calculate(double targetVelocity, double targetAcceleration)
    {
        return calculate(targetVelocity, targetAcceleration, Double.NaN);
    }

    public double calculate(double targetVelocity)
    {
        return calculate(targetVelocity, 0.0);
    }
}
