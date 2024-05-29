package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.handlers.HandleAuto;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToPropCommand extends CommandBase {

    private final AutoDriveSubsystem drive;
    private final PropProcessor processor;
    private final TrajectorySequenceBuilder leftPropPath;
    private final TrajectorySequenceBuilder centerPropPath;
    private final TrajectorySequenceBuilder rightPropPath;
    private PropProcessor.Positions propPosition;

    public GoToPropCommand(AutoDriveSubsystem drive, PropProcessor processor, boolean isRed, boolean isUp) {
        this.drive = drive;
        this.processor = processor;
        addRequirements(drive);
        if (isUp) {
            leftPropPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(isRed ? 14.00 : 11, isRed ? -34.00 : 34, Math.toRadians(isRed ? 180 : 0)), Math.toRadians(isRed ? 126 : -63));
            centerPropPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(16, isRed ? -14 : 14), Math.toRadians(isRed ? 90 : -90));
            rightPropPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(isRed ? 11 : 14, isRed ? -34.00 : 34, Math.toRadians(isRed ? 0 : 180)), Math.toRadians(isRed ? 126 : 63));
        } else {
            leftPropPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(isRed ? -37.00 : -34, isRed ? -34.00 : 34, Math.toRadians(isRed ? 0 : 180)), Math.toRadians(isRed ? 63 : -63));
            centerPropPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(-32, isRed ? -14 : 14), Math.toRadians(isRed ? 90 : -90));
            rightPropPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(isRed ? -34.00 : -37, isRed ? -34.00 : 34, Math.toRadians(isRed ? 180 : 0)), Math.toRadians(isRed ? 63 : -63));
        }
        drive.setLeftTrajectory(leftPropPath.build());
        drive.setCenterTrajectory(centerPropPath.build());
        drive.setRightTrajectory(rightPropPath.build());
    }

    @Override
    public void initialize() {
        propPosition = processor.getPropPosition();
        HandleAuto.setPropPosition(propPosition);
        if (propPosition.equals(PropProcessor.Positions.LEFT))
            drive.followTrajectory(leftPropPath.build());
        else if (propPosition.equals(PropProcessor.Positions.CENTER))
            drive.followTrajectory(centerPropPath.build());
        else
            drive.followTrajectory(rightPropPath.build());
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