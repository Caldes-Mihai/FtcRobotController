package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.util.ConstantValues;

@Config
public class RobotCentricDriveSubsystem extends SubsystemBase {
    private static CacheableMotor leftFrontDrive;
    private static CacheableMotor leftBackDrive;
    private static CacheableMotor rightFrontDrive;
    private static CacheableMotor rightBackDrive;
    public final CommandOpMode opMode;
    private final MecanumDrive drive;
    private final GamepadEx gamepad;

    public RobotCentricDriveSubsystem(HardwareMap hardwareMap, GamepadEx gamepad, CommandOpMode opMode) {
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
        this.opMode = opMode;
    }

    public void drive() {
        drive.driveRobotCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX());
    }
}