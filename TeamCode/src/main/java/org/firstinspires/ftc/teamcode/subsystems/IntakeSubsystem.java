package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double MAX_SPEED = 0.7;
    public static double MAX_SLOW_SPEED = 0.3;
    public static double SERVO_RETRACT_POS = 0;
    public static double SERVO_EXTEND_POS = 0;
    private final CacheableMotor intake;
    private final CacheableServo intake_servo;
    private final AnalogInput beam;
    private final GamepadEx gamepad;
    public int pixels = 0;
    private int stage = 5;
    private boolean oldState;
    private boolean state = false;
    private boolean slow = false;

    public IntakeSubsystem(CacheableMotor intake, CacheableServo intake_servo, AnalogInput beam) {
        this(intake, intake_servo, beam, null);
    }

    public IntakeSubsystem(CacheableMotor intake, CacheableServo intake_servo, AnalogInput beam, GamepadEx gamepad) {
        this.intake = intake;
        this.intake_servo = intake_servo;
        this.beam = beam;
        this.gamepad = gamepad;
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setReversed(boolean isReversed) {
        intake.setInverted(isReversed);
    }

    public void activate() {
        intake.set(slow ? MAX_SLOW_SPEED : MAX_SPEED);
        state = beam.getVoltage() < 1;
        if (state && !oldState)
            pixels++;
        oldState = state;
    }

    public void setSlow(boolean slow) {
        this.slow = slow;
    }

    public void deactivate() {
        intake.set(0);
    }

    public void resetPixels() {
        pixels = 0;
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