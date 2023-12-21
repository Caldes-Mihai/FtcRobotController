
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToPropCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final PropProcessor processor;
    private final boolean isRed;
    private PropProcessor.Positions propPosition;
    private double degrees;
    public GoToPropCommand(MecanumDriveSubsystem drive, PropProcessor processor, boolean isRed) {
        this.drive = drive;
        this.processor = processor;
        this.isRed = isRed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        propPosition = processor.getPropPosition();
        TrajectorySequenceBuilder propPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(27);
        if (propPosition.equals(PropProcessor.Positions.LEFT))
            degrees = Math.toRadians(-90);
        else if (propPosition.equals(PropProcessor.Positions.RIGHT))
            degrees = Math.toRadians(90);
        else
            degrees = Math.toRadians(180);
        propPath.turn(degrees);
        if(propPosition.equals(PropProcessor.Positions.CENTER))
            if (isRed) propPath.strafeLeft(3);
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