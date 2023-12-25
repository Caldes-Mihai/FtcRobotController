package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.HandleAuto;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToBoardCommand extends CommandBase {

    private final AutoDriveSubsystem drive;
    private final TrajectorySequenceBuilder leftDestinationPath;
    private final TrajectorySequenceBuilder centerDestinationPath;
    private final TrajectorySequenceBuilder rightDestinationPath;
    private PropProcessor.Positions propPosition;

    public GoToBoardCommand(AutoDriveSubsystem drive, boolean isRed, boolean isFirstTime, boolean shouldTurn) {
        this.drive = drive;
        addRequirements(drive);
        leftDestinationPath = drive.trajectorySequenceBuilder(drive.getLeftTrajectory().end());
        centerDestinationPath = drive.trajectorySequenceBuilder(drive.getCenterTrajectory().end());
        rightDestinationPath = drive.trajectorySequenceBuilder(drive.getRightTrajectory().end());
        if (isFirstTime && !shouldTurn) {
            leftDestinationPath.lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineTo(new Vector2d(36, isRed ? -31 : 41));
            centerDestinationPath.lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineTo(new Vector2d(36, isRed ? -36 : 46));
            rightDestinationPath.lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineTo(new Vector2d(36, isRed ? -41 : 31));
        } else if (shouldTurn) {
            leftDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineToLinearHeading(new Pose2d(36, isRed ? -31 : 41, drive.getLeftTrajectory().end().getHeading() + (isRed ? 0 : Math.toRadians(180))));
            centerDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineToLinearHeading(new Pose2d(36, isRed ? -36 : 36, drive.getCenterTrajectory().end().getHeading() + (isRed ? Math.toRadians(-90) : Math.toRadians(90))));
            rightDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineToLinearHeading(new Pose2d(36, isRed ? -41 : 31, drive.getRightTrajectory().end().getHeading() + (isRed ? Math.toRadians(180) : 0)));
        } else {
            leftDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12));
            centerDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12));
            rightDestinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12));
        }
        drive.setLeftTrajectory(leftDestinationPath.build());
        drive.setCenterTrajectory(centerDestinationPath.build());
        drive.setRightTrajectory(rightDestinationPath.build());
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