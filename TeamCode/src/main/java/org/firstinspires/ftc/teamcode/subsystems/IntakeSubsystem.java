package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double MAX_SPEED = 0.7;
    public static double SERVO_RETRACT_POS = 0;
    public static double SERVO_EXTEND_POS = 0;
    private final CacheableMotor intake;
    private final CacheableServo intake_servo;
    private final GamepadEx gamepad;
    private double stage = 5;

    public IntakeSubsystem(CacheableMotor intake, CacheableServo intake_servo) {
        this(intake, intake_servo, null);
    }

    public IntakeSubsystem(CacheableMotor intake, CacheableServo intake_servo, GamepadEx gamepad) {
        this.intake = intake;
        this.intake_servo = intake_servo;
        this.gamepad = gamepad;
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setReversed(boolean isReversed) {
        intake.setInverted(isReversed);
    }

    public void activate() {
        intake.set(MAX_SPEED);
    }

    public void deactivate() {
        intake.set(0);
    }

    public void extend() {
        intake_servo.setPosition(SERVO_EXTEND_POS - (SERVO_EXTEND_POS - SERVO_RETRACT_POS) / stage);
    }

    public void retract() {
        intake_servo.setPosition(SERVO_RETRACT_POS);
    }

    public void lower() {
        if (stage > 1)
            stage--;
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
        if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            this.extend();
        } else
            this.retract();
    }
}