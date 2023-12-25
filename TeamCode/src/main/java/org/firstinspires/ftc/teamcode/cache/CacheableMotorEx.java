package org.firstinspires.ftc.teamcode.cache;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CacheableMotorEx extends MotorEx {
    private double cachedPower = 0;

    public CacheableMotorEx(@NonNull HardwareMap hMap, String id) {
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
