package org.firstinspires.ftc.teamcode.commands;

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

    public GoToBoardCommand(AutoDriveSubsystem drive, boolean isRed, boolean isUp) {
        this.drive = drive;
        addRequirements(drive);
        if (isUp) {
            if (!isRed) {
                leftDestinationPath = drive.trajectorySequenceBuilder(drive.getLeftTrajectory().end())
                        .splineTo(new Vector2d(12, isRed ? -41 : 41), Math.toRadians(isRed ? -21 : 21))
                        .splineTo(new Vector2d(48.00, isRed ? -41 : 29), Math.toRadians(0.00));
                rightDestinationPath = drive.trajectorySequenceBuilder(drive.getRightTrajectory().end())
                        .splineTo(new Vector2d(48, isRed ? -29 : 41), Math.toRadians(0.00));
            } else {
                leftDestinationPath = drive.trajectorySequenceBuilder(drive.getLeftTrajectory().end())
                        .splineTo(new Vector2d(48, isRed ? -29 : 41), Math.toRadians(0.00));
                rightDestinationPath = drive.trajectorySequenceBuilder(drive.getRightTrajectory().end())
                        .splineTo(new Vector2d(12, isRed ? -41 : 41), Math.toRadians(isRed ? -21 : 21))
                        .splineTo(new Vector2d(48.00, isRed ? -41 : 29), Math.toRadians(0.00));
            }
            centerDestinationPath = drive.trajectorySequenceBuilder(drive.getCenterTrajectory().end())
                    .splineTo(new Vector2d(30, isRed ? -16 : 16), Math.toRadians(isRed ? -45.00 : 45))
                    .splineTo(new Vector2d(48, isRed ? -35 : 35), Math.toRadians(0.00));
        } else {
            leftDestinationPath = drive.trajectorySequenceBuilder(drive.getLeftTrajectory().end())
                    .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? -21 : 21))
                    .splineTo(new Vector2d(48, isRed ? -29 : 41), Math.toRadians(0.00));
            rightDestinationPath = drive.trajectorySequenceBuilder(drive.getRightTrajectory().end())
                    .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? -21 : 21))
                    .splineTo(new Vector2d(48.00, isRed ? -41 : 29), Math.toRadians(0.00));
            centerDestinationPath = drive.trajectorySequenceBuilder(drive.getCenterTrajectory().end())
                    .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? -21 : 21))
                    .splineTo(new Vector2d(48, isRed ? -35 : 35), Math.toRadians(0.00));
        }
        drive.setLeftTrajectory(leftDestinationPath.build());
        drive.setCenterTrajectory(centerDestinationPath.build());
        drive.setRightTrajectory(rightDestinationPath.build());
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