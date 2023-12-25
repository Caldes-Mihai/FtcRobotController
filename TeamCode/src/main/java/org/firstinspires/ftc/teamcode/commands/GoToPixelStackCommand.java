package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.HandleAuto;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToPixelStackCommand extends CommandBase {

    private final AutoDriveSubsystem drive;
    private final double stackY;
    private final TrajectorySequenceBuilder leftLinePath;
    private final TrajectorySequenceBuilder centerLinePath;
    private final TrajectorySequenceBuilder rightLinePath;
    private PropProcessor.Positions propPosition;

    public GoToPixelStackCommand(AutoDriveSubsystem drive, boolean isRed, boolean shouldTurn) {
        this.drive = drive;
        addRequirements(drive);
        if (isRed)
            stackY = -12;
        else
            stackY = 12;
        leftLinePath = drive.trajectorySequenceBuilder(drive.getLeftTrajectory().end());
        centerLinePath = drive.trajectorySequenceBuilder(drive.getCenterTrajectory().end());
        rightLinePath = drive.trajectorySequenceBuilder(drive.getRightTrajectory().end());
        if (!shouldTurn) {
            leftLinePath.lineTo(new Vector2d(36, stackY))
                    .lineTo(new Vector2d(-57, stackY));
            centerLinePath.lineTo(new Vector2d(36, stackY))
                    .lineTo(new Vector2d(-57, stackY));
            rightLinePath.lineTo(new Vector2d(36, stackY))
                    .lineTo(new Vector2d(-57, stackY));
        } else {
            centerLinePath.lineToLinearHeading(new Pose2d(-57, stackY, drive.getCenterTrajectory().end().getHeading() + (isRed ? Math.toRadians(-90) : Math.toRadians(90))));
            if (!isRed) {
                leftLinePath.lineToLinearHeading(new Pose2d(-57, stackY, drive.getLeftTrajectory().end().getHeading() + Math.toRadians(180)));
                rightLinePath.lineTo(new Vector2d(-57, stackY));
            } else {
                leftLinePath.lineTo(new Vector2d(-57, stackY));
                rightLinePath.lineToLinearHeading(new Pose2d(-57, stackY, drive.getRightTrajectory().end().getHeading() + Math.toRadians(180)));
            }
        }
        drive.setLeftTrajectory(leftLinePath.build());
        drive.setCenterTrajectory(centerLinePath.build());
        drive.setRightTrajectory(rightLinePath.build());
    }

    @Override
    public void initialize() {
        propPosition = HandleAuto.getPropPosition();
        if (propPosition.equals(PropProcessor.Positions.LEFT))
            drive.followTrajectory(leftLinePath.build());
        else if (propPosition.equals(PropProcessor.Positions.CENTER))
            drive.followTrajectory(centerLinePath.build());
        else
            drive.followTrajectory(rightLinePath.build());
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