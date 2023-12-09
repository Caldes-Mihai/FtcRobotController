package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem extends SubsystemBase {
    private double accel, axial, lateral, yaw;
    private MecanumDrive drive;
    private IMU imu;
    private GamepadEx gamepad;

    public DriveSubsystem(Motor frontLeftMotor, Motor backLeftMotor, Motor frontRightMotor, Motor backRightMotor, IMU imu,
                          GamepadEx gamepad) {
        drive = new MecanumDrive(false, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        this.imu = imu;
        this.gamepad = gamepad;
    }

    public void drive() {
        accel = 1 - gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        axial = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).get() ? accel : gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).get() ? -accel : 0;
        lateral = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get() ? accel : gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get() ? -accel : 0;
        yaw = gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get() ? accel : gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).get() ? -accel : 0;
        drive.driveFieldCentric(lateral, axial, yaw, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
}