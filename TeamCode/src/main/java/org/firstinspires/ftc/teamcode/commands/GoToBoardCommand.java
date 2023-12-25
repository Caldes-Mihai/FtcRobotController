package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.HandleAuto;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;
import java.util.Arrays;


public class GoToBoardCommand extends CommandBase {

    private final AutoDriveSubsystem drive;
    private final TrajectorySequenceBuilder leftDestinationPath;
    private final TrajectorySequenceBuilder centerDestinationPath;
    private final TrajectorySequenceBuilder rightDestinationPath;
    private PropProcessor.Positions propPosition;

    public GoToBoardCommand(AutoDriveSubsystem drive, boolean isRed, boolean isFirstTime, boolean shouldTurn) {
        this.drive = drive;
        addRequirements(drive);
        leftDestinationPath = drive.trajectorySequenceBuilder(drive.getLastTrajectory().get(0).end());
        if (isFirstTime && !shouldTurn)
            leftDestinationPath.lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineTo(new Vector2d(36, isRed ? -31 : 41));
        else if (shouldTurn)
            leftDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineToLinearHeading(new Pose2d(36, isRed ? -31 : 41, drive.getLastTrajectory().get(0).end().getHeading() + (isRed ? 0 : Math.toRadians(180))));
        else
            leftDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12));
        centerDestinationPath = drive.trajectorySequenceBuilder(drive.getLastTrajectory().get(1).end());
        if (isFirstTime && !shouldTurn)
            centerDestinationPath.lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineTo(new Vector2d(36, isRed ? -36 : 46));
        else if (shouldTurn)
            centerDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineToLinearHeading(new Pose2d(36, isRed ? -36 : 36, drive.getLastTrajectory().get(1).end().getHeading() + (isRed ? Math.toRadians(-90) : Math.toRadians(90))));
        else
            centerDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12));
        rightDestinationPath = drive.trajectorySequenceBuilder(drive.getLastTrajectory().get(2).end());
        if (isFirstTime && !shouldTurn)
            rightDestinationPath.lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineTo(new Vector2d(36, isRed ? -41 : 31));
        else if (shouldTurn)
            rightDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineToLinearHeading(new Pose2d(36, isRed ? -41 : 31, drive.getLastTrajectory().get(2).end().getHeading() + (isRed ? Math.toRadians(180) : 0)));
        else
            rightDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12));
        drive.setLastTrajectory(new ArrayList(Arrays.asList(leftDestinationPath.build(), centerDestinationPath.build(), rightDestinationPath.build())));
    }

    public GoToBoardCommand(AutoDriveSubsystem drive, boolean isRed, boolean isFirstTime) {
        this(drive, isRed, isFirstTime, false);
    }

    @Override
    public void initialize() {
        propPosition = HandleAuto.getPropPosition();
        if (propPosition.equals(PropProcessor.Positions.LEFT))
            drive.followTrajectory(leftDestinationPath.build());
        else if (propPosition.equals(PropProcessor.Positions.CENTER))
            drive.followTrajectory(centerDestinationPath.build());
        else
            drive.followTrajectory(rightDestinationPath.build());
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}