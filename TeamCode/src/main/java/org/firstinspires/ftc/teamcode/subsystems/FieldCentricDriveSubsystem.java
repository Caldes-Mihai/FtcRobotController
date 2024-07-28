package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.ConstantValues;

@Config
public class FieldCentricDriveSubsystem extends SubsystemBase {
    private static CacheableMotor leftFrontDrive;
    private static CacheableMotor leftBackDrive;
    private static CacheableMotor rightFrontDrive;
    private static CacheableMotor rightBackDrive;
    private final MecanumDrive drive;
    private final IMU imu;
    private final GamepadEx gamepad;
    private final boolean isRed;
    private final CommandOpMode opMode;
    private double CURRENT_SPEED = 1;
    private double imuDegrees;

    public FieldCentricDriveSubsystem(HardwareMap hardwareMap, GamepadEx gamepad, boolean isRed, CommandOpMode opMode) {
        this.imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR,
                DriveConstants.USB_FACING_DIR)));
        leftFrontDrive = new CacheableMotor(hardwareMap, "front_left_motor");
        leftBackDrive = new CacheableMotor(hardwareMap, "back_left_motor");
        rightFrontDrive = new CacheableMotor(hardwareMap, "front_right_motor");
        rightBackDrive = new CacheableMotor(hardwareMap, "back_right_motor");
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        leftFrontDrive.setInverted(ConstantValues.INVERT_LEFT_FRONT);
        leftBackDrive.setInverted(ConstantValues.INVERT_LEFT_BACK);
        rightFrontDrive.setInverted(ConstantValues.INVERT_RIGHT_FRONT);
        rightBackDrive.setInverted(ConstantValues.INVERT_RIGHT_BACK);
        this.drive = new MecanumDrive(false, leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        this.gamepad = gamepad;
        this.isRed = isRed;
        this.opMode = opMode;
    }

    public void drive() {
        if (gamepad.getButton(ConstantValues.RESET_IMU))
            imu.resetYaw();
        imuDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (gamepad.getButton(ConstantValues.PRECISION_MODE))
            CURRENT_SPEED = ConstantValues.PRECISION_MODE_SPEED;
        else
            CURRENT_SPEED = ConstantValues.NORMAL_SPEED;
        opMode.telemetry.addData("imu", imuDegrees);
        opMode.telemetry.addData("reset", gamepad.getButton(ConstantValues.RESET_IMU));
        drive.driveFieldCentric(gamepad.getLeftX() * CURRENT_SPEED, gamepad.getLeftY() * CURRENT_SPEED, gamepad.getRightX() * CURRENT_SPEED, imuDegrees + (isRed ? -90 : 90));
    }
}