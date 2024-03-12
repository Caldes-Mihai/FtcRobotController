package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.cache.CacheableServo;

@Config
public class DroneSubsystem extends SubsystemBase {
    public static double start = 0.82;
    public static double end = 0.5;
    private final CacheableServo drone;
    private final GamepadEx gamepad;

    public DroneSubsystem(HardwareMap hardwareMap, GamepadEx gamepad) {
        this.gamepad = gamepad;
        this.drone = new CacheableServo(hardwareMap, "drone", 0, 270);
    }

    public void handle() {
        if (gamepad.getButton(GamepadKeys.Button.Y))
            drone.setPosition(end);
        else
            drone.setPosition(start);
    }
}