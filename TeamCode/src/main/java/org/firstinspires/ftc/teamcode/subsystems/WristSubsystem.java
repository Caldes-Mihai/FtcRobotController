package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.cache.CacheableServo;
import org.firstinspires.ftc.teamcode.util.ConstantValues;

@Config
public class WristSubsystem extends SubsystemBase {
    private final CacheableServo claw1;
    private final CacheableServo claw2;
    private final CacheableServo claw_wrist_horizontal;

    private final CacheableServo claw_wrist_vertical;

    public WristSubsystem(HardwareMap hardwareMap) {
        this.claw1 = new CacheableServo(hardwareMap, "claw1", 0, 270);
        this.claw2 = new CacheableServo(hardwareMap, "claw2", 0, 270);
        this.claw_wrist_horizontal = new CacheableServo(hardwareMap, "claw_wrist_horizontal", 0, 270);
        this.claw_wrist_vertical = new CacheableServo(hardwareMap, "claw_wrist_vertical", 0, 270);
        claw1.setInverted(ConstantValues.INVERT_CLAW1);
        claw2.setInverted(ConstantValues.INVERT_CLAW2);
        claw_wrist_horizontal.setInverted(ConstantValues.INVERT_CLAW_WRIST_HORIZONTAL);
        claw_wrist_vertical.setInverted(ConstantValues.INVERT_CLAW_WRIST_VERTICAL);
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
        claw1.setPosition(ConstantValues.CLAW1_HOLD_POS);
    }

    public void holdClaw2() {
        claw2.setPosition(ConstantValues.CLAW2_HOLD_POS);
    }

    public void releaseClaw1() {
        claw1.setPosition(ConstantValues.CLAW1_RELEASE_POS);
    }

    public void releaseClaw2() {
        claw2.setPosition(ConstantValues.CLAW2_RELEASE_POS);
    }

    public void rightDiagonal() {
        claw_wrist_horizontal.setPosition(ConstantValues.CLAW_WRIST_RIGHT_DIAGONAL);
    }

    public void leftDiagonal() {
        claw_wrist_horizontal.setPosition(ConstantValues.CLAW_WRIST_LEFT_DIAGONAL);
    }

    public void horizontal() {
        claw_wrist_horizontal.setPosition(ConstantValues.CLAW_WRIST_HORIZONTAL);
    }

    public void vertical() {
        claw_wrist_horizontal.setPosition(ConstantValues.CLAW_WRIST_VERTICAL);
    }

    public void activate() {
        ((PwmControl) claw_wrist_vertical).setPwmEnable();
    }

    public void deactivate() {
        ((PwmControl) claw_wrist_vertical).setPwmDisable();
    }

    public void alignWithBoard() {
        claw_wrist_vertical.setPosition(ConstantValues.EXTENDED_CLAW_WRIST_VERTICAL_SERVO_POS);
    }

    public void pickup() {
        horizontal();
        claw_wrist_vertical.setPosition(ConstantValues.RETRACTED_CLAW_WRIST_VERTICAL_SERVO_POS);
    }
}