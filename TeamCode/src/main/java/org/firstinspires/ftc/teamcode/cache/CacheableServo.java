package org.firstinspires.ftc.teamcode.cache;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CacheableServo extends SimpleServo {
    private double cachedPos = 0;

    public CacheableServo(HardwareMap hw, String servoName, double minDegree, double maxDegree) {
        super(hw, servoName, minDegree, maxDegree);
    }

    @Override
    public void setPosition(double position) {
        if (position != cachedPos) {
            cachedPos = position;
            super.setPosition(position);
        }
    }
}
