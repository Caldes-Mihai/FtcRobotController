package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.AdjustPositionCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.PickupCommand;
import org.firstinspires.ftc.teamcode.commands.PlaceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.AdjustPositionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

public class HandleAuto {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private static PropProcessor processor;
    private static AprilTagProcessor aprilTagProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private static VisionPortal visionPortal;
    private static double degrees = 0;
    private static int lineY = 0;
    private static int strafe = 0;
    private static GamepadEx driver;
    private static SampleMecanumDrive drive;
    private static MecanumDriveSubsystem mecanumDriveSubsystem;
    private static AdjustPositionSubsystem adjustPositionSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static Pose2d startPose;
    private static PropProcessor.Positions propPosition;
    private static Motor intake;
    public static void init(boolean isRed, String currentSpawnPosition, CommandOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        Gamepad gamepad1 = opMode.gamepad1;
        intake = new Motor(hardwareMap, "intake");
        processor = new PropProcessor(opMode.telemetry);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "0"))
                .addProcessors(processor, aprilTagProcessor)
                .build();
        drive = new SampleMecanumDrive(hardwareMap);
        driver = new GamepadEx(gamepad1);
        processor.setRed(isRed);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(drive, false);
        adjustPositionSubsystem = new AdjustPositionSubsystem(drive, aprilTagProcessor);
        intakeSubsystem = new IntakeSubsystem(intake);
        opMode.register(mecanumDriveSubsystem, adjustPositionSubsystem, intakeSubsystem);
        adjustPositionSubsystem.setDefaultCommand(new AdjustPositionCommand(adjustPositionSubsystem));
        if (!isRed) {
            if (currentSpawnPosition.equals("down"))
                startPose = new Pose2d(-36, 63, Math.toRadians(-90));
            else
                startPose = new Pose2d(12, 63, Math.toRadians(-90));
        } else {
            if (currentSpawnPosition.equals("down"))
                startPose = new Pose2d(-36, -63, Math.toRadians(90));
            else
                startPose = new Pose2d(12, -63, Math.toRadians(90));
        }
        drive.setPoseEstimate(startPose);
        TrajectorySequenceBuilder propPath = drive.trajectorySequenceBuilder(startPose)
                .forward(27);
        propPosition = processor.getPropPosition();
        if (isRed)
            lineY = -36;
        else
            lineY = 36;
        if (propPosition.equals(PropProcessor.Positions.LEFT)) {
            degrees = Math.toRadians(90);
            strafe = 5;
        } else if (propPosition.equals(PropProcessor.Positions.CENTER)) {
            degrees = Math.toRadians(-90);
            strafe = -5;
        } else {
            if (isRed) propPath.strafeLeft(5);
            else propPath.strafeRight(5);
            strafe = 0;
        }
        double turn = 0;
        if (propPosition.equals(PropProcessor.Positions.LEFT))
            turn = isRed ? 0 : -2 * degrees;
        else if (propPosition.equals(PropProcessor.Positions.CENTER))
            turn = isRed ? 90 : -90;
        else
            turn = isRed ? -2 * degrees : 0;
        propPath.turn(degrees);
        //get pixel then place
        TrajectorySequenceBuilder linePath = drive.trajectorySequenceBuilder(propPath.build().end()).turn(turn);
        linePath.lineTo(new Vector2d(-54, lineY)).turn( Math.toRadians(180));
        TrajectorySequenceBuilder destinationPath = drive.trajectorySequenceBuilder(linePath.build().end())
                .lineTo(new Vector2d(45, isRed ? -36 : 36));
        if (strafe > 0)
            destinationPath.strafeLeft(strafe);
        else if (strafe < 0)
            destinationPath.strafeRight(strafe);
        opMode.schedule(new SequentialCommandGroup(
                new FollowTrajectorySequenceCommand(mecanumDriveSubsystem, propPath.build()),
                new PlaceCommand(),
                new FollowTrajectorySequenceCommand(mecanumDriveSubsystem, linePath.build()),
                new PickupCommand(intakeSubsystem),
                new FollowTrajectorySequenceCommand(mecanumDriveSubsystem, destinationPath.build()),
                new PlaceCommand(),
                new RunCommand(() -> visionPortal.close())
        ));
    }
}
