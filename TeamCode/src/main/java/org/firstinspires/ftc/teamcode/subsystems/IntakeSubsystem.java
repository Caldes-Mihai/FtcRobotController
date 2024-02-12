package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.cache.CacheableMotor;

public class IntakeSubsystem extends SubsystemBase {
    private final CacheableMotor intake;
    private final GamepadEx gamepad;
    private boolean isReversed;

    public IntakeSubsystem(CacheableMotor intake) {
        this(intake, null);
    }

    public IntakeSubsystem(CacheableMotor intake, GamepadEx gamepad) {
        this.intake = intake;
        this.gamepad = gamepad;
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setReversed(boolean isReversed) {
        intake.setInverted(isReversed);
        this.isReversed = isReversed;
    }

    public void activate() {
        intake.set(0.7);
    }

    public void deactivate() {
        intake.set(0);
    }

    public void handle() {
        if (gamepad.getButton(GamepadKeys.Button.X)) {
            setReversed(true);
            this.activate();
        } else if (gamepad.getButton(GamepadKeys.Button.B)) {
            setReversed(false);
            this.activate();
        } else
            this.deactivate();

    }
}