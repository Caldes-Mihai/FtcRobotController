package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.cache.CacheableServo;
import org.firstinspires.ftc.teamcode.util.ConstantValues;

@Config
public class DroneSubsystem extends SubsystemBase {
    private final CacheableServo drone;
    private final GamepadEx gamepad;

    public DroneSubsystem(HardwareMap hardwareMap, GamepadEx gamepad) {
        this.gamepad = gamepad;
        this.drone = new CacheableServo(hardwareMap, "drone", 0, 270);
    }

    public void handle() {
        if (gamepad.getButton(ConstantValues.DRONE))
            drone.setPosition(ConstantValues.DRONE_END);
        else
            drone.setPosition(ConstantValues.DRONE_START);
    }
}