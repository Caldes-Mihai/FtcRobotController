package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double MAX_SPEED = 1;
    public static double MAX_SLOW_SPEED = 0.3;
    public static double SERVO_RETRACT_POS = 0;
    public static double SERVO_EXTEND_POS = 0;
    private final CacheableMotor intake;
    private final CacheableServo intake_servo;
    private final AnalogInput beam;
    private final GamepadEx gamepad;
    public int pixels;
    private boolean state = false;
    private boolean oldState;
    private boolean slow = false;


    public IntakeSubsystem(HardwareMap hardwareMap, int pixels, GamepadEx gamepad) {
        this.intake = new CacheableMotor(hardwareMap, "intake");
        this.intake_servo = new CacheableServo(hardwareMap, "intake_servo", 0, 270);
        this.beam = hardwareMap.analogInput.get("beam");
        this.pixels = pixels;
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
        intake_servo.setPosition(SERVO_EXTEND_POS);
    }

    public void retract() {
        intake_servo.setPosition(SERVO_RETRACT_POS);
    }

    public void handle() {
        setSlow(false);
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
        if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            resetPixels();
        }
    }
}