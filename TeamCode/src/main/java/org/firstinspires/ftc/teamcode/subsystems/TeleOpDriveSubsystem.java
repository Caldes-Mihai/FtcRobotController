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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Config
public class TeleOpDriveSubsystem extends SubsystemBase {
    public static double Kp, Ki, Kd;
    public static boolean debug = false;
    public static double target = 0;
    private final MecanumDrive drive;
    private final IMU imu;
    private final GamepadEx gamepad;
    private final boolean isRed;
    private final CommandOpMode opMode;
    private final PIDController pidController = new PIDController(Kp, Ki, Kd);
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double axial, lateral, yaw, oldYaw, distance, imuDegrees, imuRadians, dif;
    private Vector2d joystick;
    private double integralSum;

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
        joystick = joystick.rotated(Math.toRadians(isRed ? 0 : 180));
        yaw = Math.atan2(-joystick.getY(), joystick.getX());
        imuDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        imuRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        distance = joystick.distTo(new Vector2d(0, 0));
        if (yaw != oldYaw && distance > 0.7)
            oldYaw = yaw;
        if (debug)
            oldYaw = Math.toRadians(target);
        dif = PIDControl(oldYaw, imuRadians);
        opMode.telemetry.addData("IMU", imuDegrees);
        opMode.telemetry.addData("TARGET", Math.toDegrees(oldYaw));
        opMode.telemetry.addData("DIF", Math.toDegrees(dif));
        opMode.telemetry.addData("joystick", Math.toDegrees(yaw));
        drive.driveFieldCentric(lateral, axial, Math.abs(Math.toDegrees(dif)) > 3 ? Range.clip(dif, -0.3, 0.3) : 0, imuDegrees + (isRed ? -90 : 90));
    }

    private double PIDControl(double reference, double state) {
        double error = angleWrap(reference - state);
        opMode.telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }

    private double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }
}