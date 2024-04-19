package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;
import org.firstinspires.ftc.teamcode.drive.ConstantValues;

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

    public void handle() {
        setSlow(false);
        if ((!pixel1 || !pixel2) && (gamepad.getButton(GamepadKeys.Button.X) || gamepad2.getButton(GamepadKeys.Button.X))) {
            setReversed(true);
            this.activate();
        } else if (gamepad.getButton(GamepadKeys.Button.B) || gamepad2.getButton(GamepadKeys.Button.B)) {
            setReversed(false);
            this.activate();
        } else
            this.deactivate();
        if (gamepad.getButton(GamepadKeys.Button.DPAD_UP) || gamepad2.getButton(GamepadKeys.Button.DPAD_UP)) {
            this.extend();
        } else
            this.retract();
    }

    @Override
    public void periodic() {
        pixel1 = !getPixel(pixel1sensor).equals(Pixels.NONE);
        pixel2 = !getPixel(pixel2sensor).equals(Pixels.NONE);
        if (pixel1 != oldPixel1 || pixel2 != oldPixel2) {
            if (gamepad != null && gamepad2 != null) {
                gamepad.gamepad.rumble(1, 1, 250);
                gamepad2.gamepad.rumble(1, 1, 250);
            }
        }
        oldPixel1 = pixel1;
        oldPixel2 = pixel2;
        if (gamepad != null && gamepad2 != null) {
            NormalizedRGBA rgba1 = pixel1sensor.getNormalizedColors();
            NormalizedRGBA rgba2 = pixel2sensor.getNormalizedColors();
            gamepad.gamepad.setLedColor(rgba1.red * 255, rgba1.green * 255, rgba1.blue * 255, 12000);
            gamepad2.gamepad.setLedColor(rgba2.red * 255, rgba2.green * 255, rgba2.blue * 255, 12000);
        }
    }

    public Pixels getPixel(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        if (withinRange(rgba.red * 255, ConstantValues.WHITE_PIXEL[0], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.green * 255, ConstantValues.WHITE_PIXEL[1], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.blue * 255, ConstantValues.WHITE_PIXEL[2], ConstantValues.PIXEL_COLOR_THRESHOLD))
            return Pixels.WHITE;
        else if (withinRange(rgba.red * 255, ConstantValues.YELLOW_PIXEL[0], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.green * 255, ConstantValues.YELLOW_PIXEL[1], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.blue * 255, ConstantValues.YELLOW_PIXEL[2], ConstantValues.PIXEL_COLOR_THRESHOLD))
            return Pixels.YELLOW;
        else if (withinRange(rgba.red * 255, ConstantValues.GREEN_PIXEL[0], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.green * 255, ConstantValues.GREEN_PIXEL[1], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.blue * 255, ConstantValues.GREEN_PIXEL[2], ConstantValues.PIXEL_COLOR_THRESHOLD))
            return Pixels.GREEN;
        else if (withinRange(rgba.red * 255, ConstantValues.PURPLE_PIXEL[0], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.green * 255, ConstantValues.PURPLE_PIXEL[1], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.blue * 255, ConstantValues.PURPLE_PIXEL[2], ConstantValues.PIXEL_COLOR_THRESHOLD))
            return Pixels.PURPLE;
        return Pixels.NONE;
    }

    private boolean withinRange(double input1, double input2, double deviation) {
        return Math.abs(input1 - input2) <= deviation;
    }

    private enum Pixels {
        WHITE,
        PURPLE,
        GREEN,
        YELLOW,
        NONE
    }

}