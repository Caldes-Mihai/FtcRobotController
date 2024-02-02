package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.cache.CacheableCRServo;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double RETRACTED_SLIDERS_POS = 0;
    public static double RETRACTED_SLIDER_SERVO_POS = 0.35;

    public static double EXTENDED_SLIDERS_POS = 0;
    public static double EXTENDED_SLIDER_SERVO_POS = 0.1;
    public static double SLIDERS_THRESHOLD = 10;
    private final CacheableMotor slider1;
    private final CacheableMotor slider2;
    private final CacheableServo slider1_servo;
    private final CacheableServo slider2_servo;
    private final CacheableCRServo holder;
    private final GamepadEx gamepad;
    private final PIDController pidController = new PIDController(Kp, Ki, Kd);
    private double factor = 1;
    private double power = 0;

    public OuttakeSubsystem(CacheableMotor slider1, CacheableMotor slider2, CacheableServo slider1_servo, CacheableServo slider2_servo, CacheableCRServo holder) {
        this(slider1, slider2, slider1_servo, slider2_servo, holder, null);
    }

    public OuttakeSubsystem(CacheableMotor slider1, CacheableMotor slider2, CacheableServo slider1_servo, CacheableServo slider2_servo, CacheableCRServo holder, GamepadEx gamepad) {
        this.slider1 = slider1;
        this.slider2 = slider2;
        this.slider1_servo = slider1_servo;
        this.slider2_servo = slider2_servo;
        this.holder = holder;
        this.gamepad = gamepad;
        slider1.setInverted(true);
        slider1_servo.setInverted(true);
        slider1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider1.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public void periodic() {
        if (slider1.getCurrentPosition() >= EXTENDED_SLIDERS_POS / 2) {
            slider1_servo.setPosition(EXTENDED_SLIDER_SERVO_POS);
            slider2_servo.setPosition(EXTENDED_SLIDER_SERVO_POS);
        } else {
            slider1_servo.setPosition(RETRACTED_SLIDER_SERVO_POS);
            slider2_servo.setPosition(RETRACTED_SLIDER_SERVO_POS);
        }
        power = pidController.calculate(slider1.getCurrentPosition());
        slider1.set(power);
        slider2.set(power);
    }

    public void extend() {
        pidController.setSetPoint(EXTENDED_SLIDERS_POS);
    }

    public void retract() {
        pidController.setSetPoint(RETRACTED_SLIDERS_POS);
    }


    public void setReversed(boolean isReversed) {
        factor = isReversed ? -1 : 1;
    }

    public void activateHolder() {
        holder.setPower(factor);
    }

    public void deactivateHolder() {
        holder.setPower(0);
    }

    public void handle() {
        pidController.setPID(Kp, Ki, Kd);
        if (this.isExtended() && gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            this.setReversed(false);
            this.activateHolder();
        } else if (!gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER))
            this.deactivateHolder();
        else if (this.isRetracted() && gamepad.getButton(GamepadKeys.Button.X)) {
            this.setReversed(true);
            this.activateHolder();
        } else if (!gamepad.getButton(GamepadKeys.Button.X))
            this.deactivateHolder();
    }

    public boolean isExtended() {
        return withinRange(slider1.getCurrentPosition(), EXTENDED_SLIDERS_POS, SLIDERS_THRESHOLD) && withinRange(slider2.getCurrentPosition(), EXTENDED_SLIDERS_POS, SLIDERS_THRESHOLD);
    }

    public boolean isRetracted() {
        return withinRange(slider1.getCurrentPosition(), RETRACTED_SLIDERS_POS, SLIDERS_THRESHOLD) && withinRange(slider2.getCurrentPosition(), RETRACTED_SLIDERS_POS, SLIDERS_THRESHOLD);
    }

    boolean withinRange(double input1, double input2, double deviation) {
        return Math.abs(input1 - input2) <= deviation;
    }
}