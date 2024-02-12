package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.cache.CacheableServo;

@Config
public class DroneSubsystem extends SubsystemBase {
    public static double start = 0.8;
    public static double end = 0.5;
    private final CacheableServo drone;
    private final GamepadEx gamepad;

    public DroneSubsystem(CacheableServo drone, GamepadEx gamepad) {
        this.gamepad = gamepad;
        this.drone = drone;
    }

    public void handle() {
        if (gamepad.getButton(GamepadKeys.Button.Y))
            drone.setPosition(end);
        else
            drone.setPosition(start);
    }
}