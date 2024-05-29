package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.handlers.HandleAuto;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToPixelStackCommand extends CommandBase {

    private final AutoDriveSubsystem drive;
    private final TrajectorySequenceBuilder leftLinePath;
    private final TrajectorySequenceBuilder centerLinePath;
    private final TrajectorySequenceBuilder rightLinePath;
    private PropProcessor.Positions propPosition;

    public GoToPixelStackCommand(AutoDriveSubsystem drive, boolean isRed, boolean isUp) {
        this.drive = drive;
        addRequirements(drive);
        if (isUp) {
            leftLinePath = drive.trajectorySequenceBuilder(drive.getLeftTrajectory().end())
                    .lineTo(new Vector2d(-36, isRed ? -21 : 21))
                    .splineToLinearHeading(new Pose2d(-61, isRed ? -12 : 12, Math.toRadians(0)), Math.toRadians(180));
            rightLinePath = drive.trajectorySequenceBuilder(drive.getRightTrajectory().end())
                    .splineToLinearHeading(new Pose2d(-61.00, isRed ? -12.00 : 12, Math.toRadians(0)), Math.toRadians(180.00));
            centerLinePath = drive.trajectorySequenceBuilder(drive.getCenterTrajectory().end())
                    .splineToLinearHeading(new Pose2d(-61, isRed ? -12 : 12, Math.toRadians(0)), Math.toRadians(180));
        } else {
            if (!isRed) {
                leftLinePath = drive.trajectorySequenceBuilder(drive.getLeftTrajectory().end())
                        .splineToLinearHeading(new Pose2d(-61.00, 12.00, Math.toRadians(0)), Math.toRadians(180.00));
                rightLinePath = drive.trajectorySequenceBuilder(drive.getRightTrajectory().end())
                        .lineTo(new Vector2d(-36, 21))
                        .splineToLinearHeading(new Pose2d(-61, 12, Math.toRadians(0)), Math.toRadians(180));
            } else {
                leftLinePath = drive.trajectorySequenceBuilder(drive.getLeftTrajectory().end())
                        .lineTo(new Vector2d(-36, isRed ? -21 : 21))
                        .splineToLinearHeading(new Pose2d(-61, isRed ? -12 : 12, Math.toRadians(0)), Math.toRadians(180));
                rightLinePath = drive.trajectorySequenceBuilder(drive.getRightTrajectory().end())
                        .splineToLinearHeading(new Pose2d(-61.00, isRed ? -12.00 : 12, Math.toRadians(0)), Math.toRadians(180.00));
            }
            centerLinePath = drive.trajectorySequenceBuilder(drive.getCenterTrajectory().end())
                    .splineToLinearHeading(new Pose2d(-61, isRed ? -12 : 12, Math.toRadians(0)), Math.toRadians(180));
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