package org.firstinspires.ftc.teamcode.cache;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class CacheableCRServo {
    private final CRServo servo;
    private double cachedPower = 0;

    public CacheableCRServo(HardwareMap hw, String servoName) {
        servo = hw.get(CRServo.class, servoName);
    }

    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        if (power != cachedPower) {
            cachedPower = power;
            servo.setPower(power);
        }
    }
}
