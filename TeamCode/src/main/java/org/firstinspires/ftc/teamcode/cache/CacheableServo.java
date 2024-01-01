package org.firstinspires.ftc.teamcode.cache;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class CacheableServo extends SimpleServo {
    private double cachedPos = Double.NaN;

    public CacheableServo(HardwareMap hw, String servoName, double minDegree, double maxDegree) {
        super(hw, servoName, minDegree, maxDegree);
    }

    @Override
    public void setPosition(double position) {
        position = Range.clip(position, 0, 1);
        if (position != cachedPos) {
            cachedPos = position;
            super.setPosition(position);
        }
    }
}
