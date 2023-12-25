package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class TeleOpDriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private final AprilTagProcessor processor;
    private final IMU imu;
    private final GamepadEx gamepad;
    private final boolean isRed;
    private final CommandOpMode opMode;
    private final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    private final double MAX_AUTO_TURN = 0.3;
    private double accel, axial, lateral, yaw;
    private AprilTagDetection desiredTag;
    private double turn;

    public TeleOpDriveSubsystem(CacheableMotor leftFrontDrive, CacheableMotor leftBackDrive, CacheableMotor rightFrontDrive, CacheableMotor rightBackDrive, IMU imu, AprilTagProcessor processor,
                                GamepadEx gamepad, boolean isRed, CommandOpMode opMode) {
        leftFrontDrive.setInverted(true);
        rightBackDrive.setInverted(true);
        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.drive = new MecanumDrive(false, leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        this.imu = imu;
        this.gamepad = gamepad;
        this.processor = processor;
        this.isRed = isRed;
        this.opMode = opMode;
    }

    public void drive() {
        if (gamepad.getButton(GamepadKeys.Button.START))
            imu.resetYaw();
        if (gamepad.getButton(GamepadKeys.Button.A)) {
            desiredTag = processor.getDetections().get(0);
            if (desiredTag != null) {
                turn = Range.clip(desiredTag.ftcPose.bearing * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                drive.driveRobotCentric(0, 0, turn);
                opMode.sleep(10);
            }
        } else {
            accel = 1 - gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            axial = gamepad.getButton(GamepadKeys.Button.DPAD_DOWN) ? accel : gamepad.getButton(GamepadKeys.Button.DPAD_UP) ? -accel : 0;
            lateral = gamepad.getButton(GamepadKeys.Button.DPAD_LEFT) ? accel : gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) ? -accel : 0;
            yaw = gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) ? accel : gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? -accel : 0;
            drive.driveFieldCentric(lateral, axial, yaw, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + (isRed ? -90 : 90));
        }
    }
}