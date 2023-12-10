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

    public DriveSubsystem(Motor leftFrontDrive, Motor leftBackDrive, Motor rightFrontDrive, Motor rightBackDrive, IMU imu,
                          GamepadEx gamepad) {
        leftFrontDrive.setInverted(true);
        rightBackDrive.setInverted(true);
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.drive = new MecanumDrive(false, leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        this.imu = imu;
        this.gamepad = gamepad;
    }

    public void drive() {
        accel = 1 - gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        axial = gamepad.getButton(GamepadKeys.Button.DPAD_DOWN) ? accel : gamepad.getButton(GamepadKeys.Button.DPAD_UP) ? -accel : 0;
        lateral = gamepad.getButton(GamepadKeys.Button.DPAD_LEFT) ? accel : gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) ? -accel : 0;
        yaw = gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) ? accel : gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? -accel : 0;
        drive.driveFieldCentric(lateral, axial, yaw, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
}