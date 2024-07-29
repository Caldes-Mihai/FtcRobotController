package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;
import org.firstinspires.ftc.teamcode.util.ConstantValues;

@Config
public class SliderSubsystem extends SubsystemBase {
    private final CacheableMotor slider1;
    private final CacheableMotor slider2;
    private final CacheableServo slider1_servo;
    private final CacheableServo slider2_servo;
    private double multiplier = 1;
    private Telemetry telemetry;

    public SliderSubsystem(HardwareMap hardwareMap) {
        this.slider1 = new CacheableMotor(hardwareMap, "slider1");
        this.slider2 = new CacheableMotor(hardwareMap, "slider2");
        this.slider1_servo = new CacheableServo(hardwareMap, "slider1_servo", 0, 270);
        this.slider2_servo = new CacheableServo(hardwareMap, "slider2_servo", 0, 270);
        slider1.setInverted(ConstantValues.INVERT_SLIDER1);
        slider2.setInverted(ConstantValues.INVERT_SLIDER2);
        slider1_servo.setInverted(ConstantValues.INVERT_SLIDER1_SERVO);
        slider2_servo.setInverted(ConstantValues.INVERT_SLIDER2_SERVO);
        slider1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider1.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setTelemetry(Telemetry t) {
        this.telemetry = t;
    }

    @Override
    public void periodic() {
        telemetry.addData("pos", Math.abs(slider1.getCurrentPosition()));
        telemetry.addData("target", ConstantValues.EXTENDED_SLIDERS_POS);
    }

    public void extend(double power) {
        multiplier = (ConstantValues.EXTENDED_SLIDERS_POS - slider1.getCurrentPosition()) / (ConstantValues.EXTENDED_SLIDERS_POS / 3);
        slider1.set(power * multiplier + 0.1);
        slider2.set(power * multiplier + 0.1);
    }

    public void standBy() {
        slider1.set(0);
        slider2.set(0);
    }

    public void retract(double power) {
        multiplier = (ConstantValues.RETRACTED_SLIDERS_POS - slider1.getCurrentPosition()) / (ConstantValues.RETRACTED_SLIDERS_POS / 3);
        slider1.set(-power * multiplier + 0.1);
        slider2.set(-power * multiplier + 0.1);
    }

    public void alignWithBoard() {
        slider1_servo.setPosition(ConstantValues.EXTENDED_SLIDER_SERVO_POS);
        slider2_servo.setPosition(ConstantValues.EXTENDED_SLIDER_SERVO_POS);
    }

    public void pickupPixel() {
        slider1_servo.setPosition(ConstantValues.PICKUP_SLIDER_SERVO_POS);
        slider2_servo.setPosition(ConstantValues.PICKUP_SLIDER_SERVO_POS);
    }

    public void retractServo() {
        slider1_servo.setPosition(ConstantValues.RETRACTED_SLIDER_SERVO_POS);
        slider2_servo.setPosition(ConstantValues.RETRACTED_SLIDER_SERVO_POS);
    }

    public boolean isExtended() {
        return ConstantValues.withinRange(Math.abs(slider1.getCurrentPosition()), ConstantValues.EXTENDED_SLIDERS_POS, ConstantValues.SLIDERS_THRESHOLD);
    }

    public boolean isRetracted() {
        return ConstantValues.withinRange(Math.abs(slider1.getCurrentPosition()), ConstantValues.RETRACTED_SLIDERS_POS, ConstantValues.SLIDERS_THRESHOLD);
    }

    public boolean isArmRetracted() {
        return slider1_servo.getPosition() == ConstantValues.RETRACTED_SLIDER_SERVO_POS;
    }
}