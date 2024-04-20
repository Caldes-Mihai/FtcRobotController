package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;
import org.firstinspires.ftc.teamcode.util.ConstantValues;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    private final CacheableMotor slider1;
    private final CacheableMotor slider2;
    private final CacheableServo slider1_servo;
    private final CacheableServo slider2_servo;
    private final CacheableServo claw1;
    private final CacheableServo claw2;
    private final IntakeSubsystem intake;
    private final GamepadEx gamepad;
    private final double power = 1;
    private final ElapsedTime timer = new ElapsedTime();
    private Telemetry telemetry;

    public OuttakeSubsystem(HardwareMap hardwareMap, IntakeSubsystem intake, GamepadEx gamepad) {
        this.slider1 = new CacheableMotor(hardwareMap, "slider1");
        this.slider2 = new CacheableMotor(hardwareMap, "slider2");
        this.slider1_servo = new CacheableServo(hardwareMap, "slider1_servo", 0, 270);
        this.slider2_servo = new CacheableServo(hardwareMap, "slider2_servo", 0, 270);
        this.claw1 = new CacheableServo(hardwareMap, "claw1", 0, 270);
        this.claw2 = new CacheableServo(hardwareMap, "claw2", 0, 270);
        this.intake = intake;
        this.gamepad = gamepad;
        slider1.setInverted(ConstantValues.INVERT_SLIDER1);
        slider2.setInverted(ConstantValues.INVERT_SLIDER2);
        slider1_servo.setInverted(ConstantValues.INVERT_SLIDER1_SERVO);
        slider2_servo.setInverted(ConstantValues.INVERT_SLIDER2_SERVO);
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
        if (Math.abs(slider1.getCurrentPosition()) >= ConstantValues.EXTENDED_SLIDERS_POS / 2) {
            slider1_servo.setPosition(ConstantValues.EXTENDED_SLIDER_SERVO_POS);
            slider2_servo.setPosition(ConstantValues.EXTENDED_SLIDER_SERVO_POS);
        } else if (intake.pixel1 && intake.pixel2) {
            slider1_servo.setPosition(ConstantValues.PICKUP_SLIDER_SERVO_POS);
            slider2_servo.setPosition(ConstantValues.PICKUP_SLIDER_SERVO_POS);
            if (!intake.oldPixel1) timer.reset();
            if (timer.seconds() >= 1) {
                hold();
            }
        } else {
            slider1_servo.setPosition(ConstantValues.RETRACTED_SLIDER_SERVO_POS);
            slider2_servo.setPosition(ConstantValues.RETRACTED_SLIDER_SERVO_POS);
        }
        telemetry.addData("pos", Math.abs(slider1.getCurrentPosition()));
        telemetry.addData("target", ConstantValues.EXTENDED_SLIDERS_POS);
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

    public void hold() {
        holdClaw1();
        holdClaw2();
    }

    public void release() {
        releaseClaw1();
        releaseClaw2();
    }

    public void holdClaw1() {
        claw1.setPosition(ConstantValues.CLAW_HOLD_POS);
    }

    public void holdClaw2() {
        claw2.setPosition(ConstantValues.CLAW_HOLD_POS);
    }

    public void releaseClaw1() {
        claw1.setPosition(ConstantValues.CLAW_RELEASE_POS);
    }

    public void releaseClaw2() {
        claw2.setPosition(ConstantValues.CLAW_RELEASE_POS);
    }

    public void handle() {
        if (gamepad.getButton(ConstantValues.CLAW_1)) {
            releaseClaw1();
        }
        if (gamepad.getButton(ConstantValues.CLAW_2)) {
            releaseClaw2();
        }
        if (gamepad.getTrigger(ConstantValues.EXTEND_OUTTAKE) > 0.3 && !isExtended()) {
            extend();
        } else if (gamepad.getTrigger(ConstantValues.RETRACT_OUTTAKE) > 0.3 && !isRetracted())
            retract();
        else {
            standBy();
        }
    }

    public boolean isExtended() {
        return ConstantValues.withinRange(Math.abs(slider1.getCurrentPosition()), ConstantValues.EXTENDED_SLIDERS_POS, ConstantValues.SLIDERS_THRESHOLD);
    }

    public boolean isRetracted() {
        return ConstantValues.withinRange(Math.abs(slider1.getCurrentPosition()), ConstantValues.RETRACTED_SLIDERS_POS, ConstantValues.SLIDERS_THRESHOLD);
    }
}