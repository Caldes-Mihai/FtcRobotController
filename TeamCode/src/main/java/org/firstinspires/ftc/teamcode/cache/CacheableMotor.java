package org.firstinspires.ftc.teamcode.cache;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CacheableMotor extends Motor {
    private double cachedPower = 0;

    public CacheableMotor(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);
    }

    @Override
    public void set(double output) {
        if (output != cachedPower) {
            cachedPower = output;
            super.set(output);
        }
    }
}
