package org.firstinspires.ftc.teamcode.cache;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

public class CacheableMotor extends Motor {
    private double cachedPower = 0;

    public CacheableMotor(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);
        MotorConfigurationType config = this.motor.getMotorType().clone();
        config.setAchieveableMaxRPMFraction(1);
        this.motor.setMotorType(config);
    }

    public void set(double output) {
        output = Range.clip(output, -1, 1);
        if (output != cachedPower) {
            cachedPower = output;
            super.set(output);
        }
    }
}
