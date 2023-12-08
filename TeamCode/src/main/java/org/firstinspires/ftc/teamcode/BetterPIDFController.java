package org.firstinspires.ftc.teamcode;

import java.util.function.DoubleSupplier;

public class BetterPIDFController {
    public DoubleSupplier p = () -> 0, i = () -> 0, d = () -> 0, f = () -> 0;
    public double period = Double.NaN;
    public double maxOutput = 1.0;
    public double outputMulti = 1.0;

    public double pidTolerance = 0;
    public double fTolerance = 1e-6;

    public double maxIntegral = 0.25;
    public double stabilityThresh = Double.POSITIVE_INFINITY;
    public double lowPassGain = 0.2;

    private double dt = Double.NaN, error = Double.NaN;

    private double lastError = Double.NaN;
    private double integralSum = 0, errorDerivative = 0;
    private long lastUpdateTimestamp = -1;

    public BetterPIDFController() {}

    public void setPIDFCoefficients(DoubleSupplier p, DoubleSupplier i, DoubleSupplier d, DoubleSupplier f)
    {
        setPIDFCoefficients(p, i, d);
        this.f = f;
    }

    public void setPIDFCoefficients(DoubleSupplier p, DoubleSupplier i, DoubleSupplier d)
    {
        setPIDFCoefficients(p);
        this.i = i;
        this.d = d;
    }

    public void setPIDFCoefficients(DoubleSupplier p)
    {
        this.p = p;
    }

    public void clearError()
    {
        integralSum = 0;
        errorDerivative = 0;
        lastError = Double.NaN;
        lastUpdateTimestamp = -1;
    }

    public double calculate(double current, double target)
    {
        if (Double.isNaN(current) || Double.isNaN(target)) return 0.0;
        if (Double.isNaN(period)) {
            error = target - current;
        }
        else {
            error = ((target - current) % period + period) % period;
            if (period - error < error) {
                error = error - period;
            }
        }
        if (Double.isNaN(error)) return 0.0;
        long nsNow = System.nanoTime();
        if (lastUpdateTimestamp < 0) {
            dt = Double.NaN;
        }
        else {
            dt = ((double)(nsNow - lastUpdateTimestamp)) / 1e9;
        }
        calculateErrorDerivative();
        updateIntegralSum();
        double output = 0.0;
        if (Math.abs(error) > pidTolerance) output = output + p.getAsDouble() * error + i.getAsDouble() * integralSum + d.getAsDouble() * errorDerivative;
        if (Math.abs(error) > fTolerance) output = output + f.getAsDouble() * Math.signum(error);
        output *= outputMulti;
        if (Double.isNaN(output)) output = 0.0;
        output = Math.max(-maxOutput, Math.min(maxOutput, output));
        lastError = error;
        lastUpdateTimestamp = nsNow;
        return output;
    }

    private void calculateErrorDerivative()
    {
        if (Double.isNaN(dt) || Double.isNaN(error) || Double.isNaN(lastError)) {
            errorDerivative = 0;
            return;
        }
        double newDerivative = (error - lastError) / dt;
        if (!Double.isNaN(newDerivative)) {
            errorDerivative = lowPassGain * errorDerivative + (1.0 - lowPassGain) * newDerivative;
        }
        if (Double.isNaN(errorDerivative)) errorDerivative = 0;
    }

    private void updateIntegralSum()
    {
        if (Double.isNaN(dt) || Double.isNaN(error) || Double.isNaN(lastError)) return;
        if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
            integralSum = 0;
        }
        if (!Double.isNaN(errorDerivative) && Math.abs(errorDerivative) > stabilityThresh) return;
        integralSum = integralSum + dt * (error + lastError) / 2.0;
        double iVal = i.getAsDouble();
        if (integralSum * iVal > maxIntegral) {
            integralSum = maxIntegral / iVal;
        }
        if (integralSum * iVal < -maxIntegral) {
            integralSum = -maxIntegral / iVal;
        }
        if (Double.isNaN(integralSum)) integralSum = 0;
    }
}
