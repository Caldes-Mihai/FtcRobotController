package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.HandleAuto;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToPropCommand extends CommandBase {

    private final AutoDriveSubsystem drive;
    private final PropProcessor processor;
    private final boolean isRed;
    private PropProcessor.Positions propPosition;
    private double degrees;
    private double distance = 27;

    public GoToPropCommand(AutoDriveSubsystem drive, PropProcessor processor, boolean isRed) {
        this.drive = drive;
        this.processor = processor;
        this.isRed = isRed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        propPosition = processor.getPropPosition();
        HandleAuto.setPropPosition(propPosition);
        TrajectorySequenceBuilder propPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate());
        if (propPosition.equals(PropProcessor.Positions.LEFT))
            degrees = Math.toRadians(-90);
        else if (propPosition.equals(PropProcessor.Positions.RIGHT))
            degrees = Math.toRadians(90);
        if (propPosition.equals(PropProcessor.Positions.CENTER))
            distance += 25;
        propPath.back(distance);
        if (degrees != 0) propPath.turn(degrees);
        else if (isRed) propPath.strafeLeft(3);
        else propPath.strafeRight(3);
        drive.followTrajectory(propPath.build());
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