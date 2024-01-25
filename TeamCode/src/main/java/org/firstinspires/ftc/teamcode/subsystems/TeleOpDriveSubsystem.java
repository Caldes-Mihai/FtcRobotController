package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
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
    private static double Kp, Ki, Kd;
    private final MecanumDrive drive;
    private final IMU imu;
    private final GamepadEx gamepad;
    private final boolean isRed;
    private final CommandOpMode opMode;
    private final PIDController pidController = new PIDController(Kp, Ki, Kd);
    private double axial, lateral, yaw, oldYaw, distance, imuDegrees, imuRadians, dif;
    private Vector2d joystick;

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
        axial = -gamepad.getLeftY();
        lateral = gamepad.getLeftX();
        joystick = new Vector2d(gamepad.getRightX(), gamepad.getRightY());
        joystick = joystick.rotated(Math.toRadians(90));
        yaw = gamepad.getRightX();
        imuDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        imuRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        distance = joystick.distTo(new Vector2d(0, 0));
        if (yaw != oldYaw && distance > 0.7)
            oldYaw = yaw;
        pidController.setPID(Kp, Ki, Kd);
        pidController.setSetPoint(oldYaw);
        dif = angleWrap(pidController.calculate(imuRadians));
        opMode.telemetry.addData("IMU", imuDegrees);
        opMode.telemetry.addData("TARGET", Math.toDegrees(oldYaw));
        opMode.telemetry.addData("DIF", Math.toDegrees(dif));
        drive.driveFieldCentric(lateral, axial, dif, imuDegrees + (isRed ? -90 : 90));
    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }
}