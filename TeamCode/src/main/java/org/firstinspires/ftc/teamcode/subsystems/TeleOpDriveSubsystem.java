package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Config
public class TeleOpDriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private final IMU imu;
    private final GamepadEx gamepad;
    private final boolean isRed;
    private final CommandOpMode opMode;
    private double axial, lateral, imuDegrees;

    public TeleOpDriveSubsystem(CacheableMotor leftFrontDrive, CacheableMotor leftBackDrive, CacheableMotor rightFrontDrive, CacheableMotor rightBackDrive, IMU imu,
                                GamepadEx gamepad, boolean isRed, CommandOpMode opMode) {
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setInverted(DriveConstants.INVERT_LEFT_FRONT);
        leftBackDrive.setInverted(DriveConstants.INVERT_LEFT_BACK);
        rightFrontDrive.setInverted(DriveConstants.INVERT_RIGHT_FRONT);
        rightBackDrive.setInverted(DriveConstants.INVERT_RIGHT_BACK);
        this.drive = new MecanumDrive(false, leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        this.imu = imu;
        this.gamepad = gamepad;
        this.isRed = isRed;
        this.opMode = opMode;
    }

    public void drive() {
        if (gamepad.getButton(GamepadKeys.Button.START))
            imu.resetYaw();
        axial = gamepad.getLeftY();
        lateral = gamepad.getLeftX();
        imuDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        drive.driveFieldCentric(lateral, axial, gamepad.getRightX(), imuDegrees + (isRed ? -90 : 90));
    }
}