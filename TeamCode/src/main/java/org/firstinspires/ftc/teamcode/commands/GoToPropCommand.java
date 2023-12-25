package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.HandleAuto;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;
import java.util.Arrays;


public class GoToPropCommand extends CommandBase {

    private final AutoDriveSubsystem drive;
    private final PropProcessor processor;
    private final TrajectorySequenceBuilder leftPropPath;
    private final TrajectorySequenceBuilder centerPropPath;
    private final TrajectorySequenceBuilder rightPropPath;
    private PropProcessor.Positions propPosition;

    public GoToPropCommand(AutoDriveSubsystem drive, PropProcessor processor, boolean isRed) {
        this.drive = drive;
        this.processor = processor;
        addRequirements(drive);
        leftPropPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(27)
                .turn(Math.toRadians(-90));
        centerPropPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(52);
        if (isRed) centerPropPath.strafeLeft(3);
        else centerPropPath.strafeRight(3);
        rightPropPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(27)
                .turn(Math.toRadians(90));
        drive.setLastTrajectory(new ArrayList(Arrays.asList(leftPropPath.build(), centerPropPath.build(), rightPropPath.build())));
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