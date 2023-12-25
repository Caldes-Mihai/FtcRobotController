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
        leftLinePath = drive.trajectorySequenceBuilder(drive.getLastTrajectory().get(0).end());
        if (!shouldTurn)
            leftLinePath.lineTo(new Vector2d(36, stackY))
                    .lineTo(new Vector2d(-57, stackY));
        else if (!isRed)
            leftLinePath.lineToLinearHeading(new Pose2d(-57, stackY, drive.getLastTrajectory().get(0).end().getHeading() + Math.toRadians(180)));
        else
            leftLinePath.lineTo(new Vector2d(-57, stackY));
        centerLinePath = drive.trajectorySequenceBuilder(drive.getLastTrajectory().get(1).end());
        if (!shouldTurn)
            centerLinePath.lineTo(new Vector2d(36, stackY))
                    .lineTo(new Vector2d(-57, stackY));
        else
            centerLinePath.lineToLinearHeading(new Pose2d(-57, stackY, drive.getLastTrajectory().get(1).end().getHeading() + (isRed ? Math.toRadians(-90) : Math.toRadians(90))));
        rightLinePath = drive.trajectorySequenceBuilder(drive.getLastTrajectory().get(2).end());
        if (!shouldTurn)
            rightLinePath.lineTo(new Vector2d(36, stackY))
                    .lineTo(new Vector2d(-57, stackY));
        else if (isRed)
            rightLinePath.lineToLinearHeading(new Pose2d(-57, stackY, drive.getLastTrajectory().get(2).end().getHeading() + Math.toRadians(180)));
        else
            leftLinePath.lineTo(new Vector2d(-57, stackY));
        drive.setLastTrajectory(new ArrayList(Arrays.asList(leftLinePath.build(), centerLinePath.build(), rightLinePath.build())));
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