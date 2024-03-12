package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Wheels;

@Config
public class TeleOpDriveSubsystem extends SubsystemBase {
    public static boolean debug = false;
    public static double target = 0;
    public static double Kp = 0.01;
    private static CacheableMotor leftFrontDrive;
    private static CacheableMotor leftBackDrive;
    private static CacheableMotor rightFrontDrive;
    private static CacheableMotor rightBackDrive;
    private final MecanumDrive drive;
    private final IMU imu;
    private final GamepadEx gamepad;
    private final boolean isRed;
    private final CommandOpMode opMode;
    private double yaw, oldYaw, distance, dif;
    private double imuDegrees;
    private Vector2d joystick;

    public TeleOpDriveSubsystem(HardwareMap hardwareMap, GamepadEx gamepad, boolean isRed, CommandOpMode opMode) {
        this.imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR,
                DriveConstants.USB_FACING_DIR)));
        leftFrontDrive = new CacheableMotor(hardwareMap, "front_left_motor");
        leftBackDrive = new CacheableMotor(hardwareMap, "back_left_motor");
        rightFrontDrive = new CacheableMotor(hardwareMap, "front_right_motor");
        rightBackDrive = new CacheableMotor(hardwareMap, "back_right_motor");
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setInverted(Wheels.INVERT_LEFT_FRONT);
        leftBackDrive.setInverted(Wheels.INVERT_LEFT_BACK);
        rightFrontDrive.setInverted(Wheels.INVERT_RIGHT_FRONT);
        rightBackDrive.setInverted(Wheels.INVERT_RIGHT_BACK);
        this.drive = new MecanumDrive(false, leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        this.gamepad = gamepad;
        this.isRed = isRed;
        this.opMode = opMode;
    }

    public void drive() {
        if (gamepad.getButton(GamepadKeys.Button.START))
            imu.resetYaw();
        imuDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        joystick = new Vector2d(gamepad.getRightX(), gamepad.getRightY());
        joystick = joystick.rotated(Math.toRadians(isRed ? 0 : 180));
        yaw = Math.toDegrees(Math.atan2(-joystick.getY(), joystick.getX()));
        distance = joystick.distTo(new Vector2d(0, 0));
        if (yaw != oldYaw && distance > 0.7)
            oldYaw = yaw;
        if (debug)
            oldYaw = target;
        dif = angleWrap(oldYaw - imuDegrees);
        drive.driveFieldCentric(gamepad.getLeftX(), gamepad.getLeftY(), Math.abs(dif) > 3 ? Range.clip(dif * Kp, -0.3, 0.3) : 0, imuDegrees + (isRed ? -90 : 90));
    }

    private double angleWrap(double headingError) {
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        return headingError;
    }
}