package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cache.CacheableCRServo;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    public static double RETRACTED_SLIDERS_POS = 0;
    public static double RETRACTED_SLIDER_SERVO_POS = 0.7;

    public static double EXTENDED_SLIDERS_POS = 3200;
    public static double EXTENDED_SLIDER_SERVO_POS = 0.55;
    public static double SLIDERS_THRESHOLD = 200;
    private final CacheableMotor slider1;
    private final CacheableMotor slider2;
    private final CacheableServo slider1_servo;
    private final CacheableServo slider2_servo;
    private final CacheableCRServo holder;
    private final GamepadEx gamepad;
    private final double power = 1;
    private double factor = 1;
    private Telemetry telemetry;

    public OuttakeSubsystem(HardwareMap hardwareMap, GamepadEx gamepad) {
        this.slider1 = new CacheableMotor(hardwareMap, "slider1");
        this.slider2 = new CacheableMotor(hardwareMap, "slider2");
        this.slider1_servo = new CacheableServo(hardwareMap, "slider1_servo", 0, 270);
        this.slider2_servo = new CacheableServo(hardwareMap, "slider2_servo", 0, 270);
        this.holder = new CacheableCRServo(hardwareMap, "holder");
        this.gamepad = gamepad;
        slider1.setInverted(true);
        slider1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider1.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setTelemetry(Telemetry t) {
        this.telemetry = t;
    }

    @Override
    public void periodic() {
        if (Math.abs(slider1.getCurrentPosition()) >= 1800) {
            slider1_servo.setPosition(EXTENDED_SLIDER_SERVO_POS);
            slider2_servo.setPosition(EXTENDED_SLIDER_SERVO_POS);
        } else {
            slider1_servo.setPosition(RETRACTED_SLIDER_SERVO_POS);
            slider2_servo.setPosition(RETRACTED_SLIDER_SERVO_POS);
        }
        telemetry.addData("pos", Math.abs(slider1.getCurrentPosition()));
        telemetry.addData("target", EXTENDED_SLIDERS_POS);
    }

    public void extend() {
        slider1.set(power);
        slider2.set(power);
    }

    public void standBy() {
        slider1.set(0);
        slider2.set(0);
    }

    public void retract() {
        slider1.set(-power);
        slider2.set(-power);
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
        if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            this.setReversed(false);
            this.activateHolder();
        }
        if (gamepad.getButton(GamepadKeys.Button.X)) {
            this.setReversed(true);
            this.activateHolder();
        }
        if (!gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) && !gamepad.getButton(GamepadKeys.Button.X))
            this.deactivateHolder();
        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3 && !isExtended()) {
            extend();
        } else if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3 && !isRetracted())
            retract();
        else {
            standBy();
        }
    }

    public boolean isExtended() {
        return withinRange(Math.abs(slider1.getCurrentPosition()), EXTENDED_SLIDERS_POS, SLIDERS_THRESHOLD);
    }

    public boolean isRetracted() {
        return withinRange(Math.abs(slider1.getCurrentPosition()), RETRACTED_SLIDERS_POS, SLIDERS_THRESHOLD);
    }

    boolean withinRange(double input1, double input2, double deviation) {
        return Math.abs(input1 - input2) <= deviation;
    }
}