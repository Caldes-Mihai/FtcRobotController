
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToPropCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final PropProcessor.Positions propPosition;
    private final boolean isRed;
    private double degrees;
    public GoToPropCommand(MecanumDriveSubsystem drive, PropProcessor.Positions propPosition, boolean isRed) {
        this.drive = drive;
        this.propPosition = propPosition;
        this.isRed = isRed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        TrajectorySequenceBuilder propPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(27);
        if (propPosition.equals(PropProcessor.Positions.LEFT))
            degrees = Math.toRadians(90);
        else if (propPosition.equals(PropProcessor.Positions.CENTER))
            degrees = Math.toRadians(-90);
        else
            if (isRed) propPath.strafeLeft(5);
            else propPath.strafeRight(5);
        propPath.turn(degrees);
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