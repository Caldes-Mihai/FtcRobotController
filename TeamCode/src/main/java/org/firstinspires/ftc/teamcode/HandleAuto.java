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
import org.firstinspires.ftc.teamcode.commands.GoToBoardCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPixelStackCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPropCommand;
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
    private static SampleMecanumDrive drive;
    private static MecanumDriveSubsystem mecanumDriveSubsystem;
    private static AdjustPositionSubsystem adjustPositionSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static Pose2d startPose;
    private static PropProcessor.Positions propPosition;
    private static Motor intake;
    public static void init(boolean isRed, String currentSpawnPosition, CommandOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
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
        propPosition = processor.getPropPosition();
        opMode.schedule(new SequentialCommandGroup(
                new GoToPropCommand(mecanumDriveSubsystem, propPosition, isRed),
                new PlaceCommand(),
                new GoToPixelStackCommand(mecanumDriveSubsystem, propPosition, isRed),
                new PickupCommand(intakeSubsystem),
                new GoToBoardCommand(mecanumDriveSubsystem, propPosition, isRed),
                new PlaceCommand(),
                new RunCommand(() -> visionPortal.close())
        ));
    }
}
