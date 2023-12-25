package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.cache.CacheableMotor;

public class IntakeSubsystem extends SubsystemBase {
    private final CacheableMotor intake;
    private final GamepadEx gamepad;
    private double factor = 1;

    public IntakeSubsystem(CacheableMotor intake) {
        this(intake, null);
    }

    public IntakeSubsystem(CacheableMotor intake, GamepadEx gamepad) {
        this.intake = intake;
        this.gamepad = gamepad;
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setReversed(boolean isReversed) {
        factor = isReversed ? -1 : 1;
    }

    public void activate() {
        intake.set(factor);
    }

    public void deactivate() {
        intake.set(0);
    }

    public void handle() {
        if (gamepad.getButton(GamepadKeys.Button.X))
            this.activate();
        else
            this.deactivate();
    }
}