package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;
import org.firstinspires.ftc.teamcode.util.ConstantValues;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final CacheableMotor intake;
    private final CacheableServo intake_servo;
    private final NormalizedColorSensor pixel1sensor;
    private final NormalizedColorSensor pixel2sensor;
    private final GamepadEx gamepad;
    private final GamepadEx gamepad2;
    public boolean pixel1;
    public boolean pixel2;
    public boolean oldPixel1;
    public boolean oldPixel2;
    private boolean slow = false;

    public IntakeSubsystem(HardwareMap hardwareMap, GamepadEx gamepad, GamepadEx gamepad2) {
        this.intake = new CacheableMotor(hardwareMap, "intake");
        this.intake_servo = new CacheableServo(hardwareMap, "intake_servo", 0, 270);
        this.pixel1sensor = hardwareMap.get(NormalizedColorSensor.class, "pixel1");
        this.pixel2sensor = hardwareMap.get(NormalizedColorSensor.class, "pixel2");
        this.gamepad = gamepad;
        this.gamepad2 = gamepad2;
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setReversed(boolean isReversed) {
        intake.setInverted(isReversed);
    }

    public void activate() {
        intake.set(slow ? ConstantValues.INTAKE_MAX_SLOW_SPEED : ConstantValues.INTAKE_MAX_SPEED);
    }

    public void setSlow(boolean slow) {
        this.slow = slow;
    }

    public void deactivate() {
        intake.set(0);
    }

    public void extend() {
        intake_servo.setPosition(ConstantValues.INTAKE_SERVO_EXTEND_POS);
    }

    public void retract() {
        intake_servo.setPosition(ConstantValues.INTAKE_SERVO_RETRACT_POS);
    }

    @Override
    public void periodic() {
        pixel1 = getPixel(pixel1sensor);
        pixel2 = getPixel(pixel2sensor);
        if ((pixel1 != oldPixel1 && !oldPixel1) || (pixel2 != oldPixel2 && !oldPixel2)) {
            if (gamepad != null && gamepad2 != null) {
                gamepad.gamepad.rumble(1, 1, 250);
                gamepad2.gamepad.rumble(1, 1, 250);
            }
        }
        oldPixel1 = pixel1;
        oldPixel2 = pixel2;
    }

    public boolean getPixel(NormalizedColorSensor sensor) {
        if (sensor == null) return false;
        return ((DistanceSensor) sensor).getDistance(DistanceUnit.CM) < ConstantValues.PIXEL_DISTANCE_THRESHOLD;
    }
}