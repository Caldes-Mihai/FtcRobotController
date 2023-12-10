package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.AdjustPositionCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.PickupCommand;
import org.firstinspires.ftc.teamcode.commands.PlaceCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.AdjustPositionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Example VisionPortal OpMode")
public class AutoDriveToProp extends CommandOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private PropProcessor processor;
    private AprilTagProcessor aprilTagProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private List<String> spawnPositions = Arrays.asList("down", "up");
    private String currentSpawnPosition = spawnPositions.get(0);
    private boolean isRed;
    private double degrees = 0;
    int lineY = 0;
    int strafe = 0;
    private GamepadEx driver;
    private SampleMecanumDrive drive;
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private AdjustPositionSubsystem adjustPositionSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private Pose2d startPose;
    private PropProcessor.Positions propPosition;
    private Motor intake;
    @Override
    public void initialize() {
        intake = new Motor(hardwareMap, "intake");
        processor = new PropProcessor(telemetry);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "0"))
                .addProcessors(processor, aprilTagProcessor)
                .build();
        drive = new SampleMecanumDrive(hardwareMap);
        driver = new GamepadEx(gamepad1);
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenReleased(() -> {
            int pos = spawnPositions.indexOf(currentSpawnPosition);
            if (pos != 0)
                currentSpawnPosition = spawnPositions.get(pos - 1);
        });
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenReleased(() -> {
            int pos = spawnPositions.indexOf(currentSpawnPosition);
            if (pos != spawnPositions.size() - 1)
                currentSpawnPosition = spawnPositions.get(pos + 1);
        });
        driver.getGamepadButton(GamepadKeys.Button.A).whenReleased(() -> isRed = !isRed);
        processor.setRed(isRed);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(drive, false);
        adjustPositionSubsystem = new AdjustPositionSubsystem(drive, aprilTagProcessor);
        intakeSubsystem = new IntakeSubsystem(intake);
        register(mecanumDriveSubsystem, adjustPositionSubsystem, intakeSubsystem);
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
        TrajectoryBuilder propPath = drive.trajectoryBuilder(startPose)
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
        TurnCommand turn = null;
        if (propPosition.equals(PropProcessor.Positions.LEFT))
            turn = new TurnCommand(mecanumDriveSubsystem, isRed ? 0 : -2 * degrees);
        else if (propPosition.equals(PropProcessor.Positions.CENTER))
            turn = new TurnCommand(mecanumDriveSubsystem, isRed ? 90 : -90);
        else
            turn = new TurnCommand(mecanumDriveSubsystem, isRed ? -2 * degrees : 0);
        //get pixel then place
        TrajectoryBuilder linePath = drive.trajectoryBuilder(propPath.build().end());
        linePath.lineTo(new Vector2d(-54, lineY));
        TrajectoryBuilder destinationPath = drive.trajectoryBuilder(linePath.build().end())
                .lineTo(new Vector2d(45, isRed ? -36 : 36));
        if (strafe > 0)
            destinationPath.strafeLeft(strafe);
        else if (strafe < 0)
            destinationPath.strafeRight(strafe);
        schedule(new SequentialCommandGroup(
                new FollowTrajectoryCommand(mecanumDriveSubsystem, propPath.build()),
                new TurnCommand(mecanumDriveSubsystem, degrees),
                new PlaceCommand(),
                turn,
                new FollowTrajectoryCommand(mecanumDriveSubsystem, linePath.build()),
                new TurnCommand(mecanumDriveSubsystem, Math.toRadians(180)),
                new PickupCommand(intakeSubsystem),
                new FollowTrajectoryCommand(mecanumDriveSubsystem, destinationPath.build()),
                new PlaceCommand(),
                new RunCommand(() -> visionPortal.close())
        ));
    }
}
