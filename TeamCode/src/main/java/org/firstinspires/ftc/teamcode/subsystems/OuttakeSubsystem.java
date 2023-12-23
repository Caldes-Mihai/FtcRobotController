package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class OuttakeSubsystem extends SubsystemBase {
    private final Motor sliders;
    private final GamepadEx gamepad;

    public OuttakeSubsystem(Motor sliders) {
        this(sliders, null);
    }

    public OuttakeSubsystem(Motor sliders, GamepadEx gamepad) {
        //TODO: Add holder motor and change sliders motor to servo depending on which one we will use
        this.sliders = sliders;
        this.gamepad = gamepad;
        sliders.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void extend() {
        sliders.set(1);
    }

    public void retract() {
        sliders.set(-1);
    }

    public void handle() {
        if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            this.extend();
        } else if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            //TODO: also release pixel from holder
            this.retract();
        }
    }
}