package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;

public class OuttakeSubsystem extends SubsystemBase {
    private final double HOLDER_ANGLE = 180;
    private final double SLIDERS_POS = 180;
    private final double Kg = 1;
    private final double Kp = 0;
    private final double Ki = 0;
    private final double Kd = 0;
    private final CacheableMotor sliders;
    private final CacheableServo holder;
    private final GamepadEx gamepad;
    private final PIDController pidController;

    public OuttakeSubsystem(CacheableMotor sliders, CacheableServo holder) {
        this(sliders, holder, null);
    }

    public OuttakeSubsystem(CacheableMotor sliders, CacheableServo holder, GamepadEx gamepad) {
        //TODO: Change motors depending on which one we will use
        this.sliders = sliders;
        this.holder = holder;
        this.gamepad = gamepad;
        this.pidController = new PIDController(Kp, Ki, Kd);
    }

    @Override
    public void periodic() {
        sliders.set(pidController.calculate(sliders.getCurrentPosition()) + Kg);
    }

    public void extend() {
        pidController.setSetPoint(SLIDERS_POS);
    }

    public void retract() {
        pidController.setSetPoint(0);
    }

    public void release() {
        holder.rotateByAngle(HOLDER_ANGLE);
    }

    public void resetHolder() {
        holder.turnToAngle(0);
    }

    public void handle() {
        if (this.isExtended() && gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
            this.release();
        else if (holder.getAngle() != 0)
            this.resetHolder();

        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3)
            this.extend();
        else
            this.retract();
    }

    public boolean isExtended() {
        return sliders.getCurrentPosition() == SLIDERS_POS;
    }

    public boolean isRetracted() {
        return sliders.getCurrentPosition() == 0;
    }

    public boolean isReleased() {
        return holder.getAngle() >= HOLDER_ANGLE;
    }

    public boolean isReset() {
        return holder.getAngle() == 0;
    }
}