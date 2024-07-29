package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.cache.CacheableServo;
import org.firstinspires.ftc.teamcode.util.ConstantValues;

@Config
public class DroneSubsystem extends SubsystemBase {
    private final CacheableServo drone;

    public DroneSubsystem(HardwareMap hardwareMap) {
        this.drone = new CacheableServo(hardwareMap, "drone", 0, 270);
    }

    public void launch() {
        drone.setPosition(ConstantValues.DRONE_END);
    }

    public void reset() {
        drone.setPosition(ConstantValues.DRONE_START);
    }
}