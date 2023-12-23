package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;

public class OuttakeSubsystem extends SubsystemBase {
    private final double HOLDER_ANGLE = 180;
    private final double SLIDERS_ANGLE = 180;
    private final ServoEx sliders;
    private final ServoEx holder;
    private final GamepadEx gamepad;

    public OuttakeSubsystem(ServoEx sliders, ServoEx holder) {
        this(sliders, holder, null);
    }

    public OuttakeSubsystem(ServoEx sliders, ServoEx holder, GamepadEx gamepad) {
        //TODO: Change motors depending on which one we will use
        this.sliders = sliders;
        this.holder = holder;
        this.gamepad = gamepad;
    }

    public void extend() {
        sliders.turnToAngle(SLIDERS_ANGLE);
    }

    public void retract() {
        sliders.turnToAngle(0);
    }

    public void release() {
        holder.turnToAngle(HOLDER_ANGLE);
    }

    public void resetHolder() {
        holder.turnToAngle(0);
    }

    public void handle() {
        if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            this.extend();
        } else if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            this.release();
            if (holder.getAngle() == HOLDER_ANGLE)
                this.resetHolder();
            this.retract();
        }
    }

    public boolean isExtended() {
        return sliders.getAngle() == SLIDERS_ANGLE;
    }

    public boolean isRetracted() {
        return sliders.getAngle() == 0;
    }

    public boolean isReleased() {
        return holder.getAngle() == HOLDER_ANGLE;
    }

    public boolean isReset() {
        return holder.getAngle() == 0;
    }
}