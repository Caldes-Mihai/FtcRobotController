
package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToPixelStackCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final PropProcessor.Positions propPosition;
    private final boolean isRed;
    private double turn;
    private double stackY;
    public GoToPixelStackCommand(MecanumDriveSubsystem drive, PropProcessor.Positions propPosition, boolean isRed) {
        this.drive = drive;
        this.propPosition = propPosition;
        this.isRed = isRed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        if (isRed)
            stackY = -36;
        else
            stackY = 36;
        if (propPosition.equals(PropProcessor.Positions.LEFT))
            turn = isRed ? 0 : Math.toRadians(180);
        else if (propPosition.equals(PropProcessor.Positions.CENTER))
            turn = isRed ? 90 : -90;
        else
            turn = isRed ? Math.toRadians(180) : 0;
        TrajectorySequenceBuilder linePath = drive.trajectorySequenceBuilder(drive.getPoseEstimate()).turn(turn);
        linePath.lineTo(new Vector2d(-54, stackY)).turn( Math.toRadians(180));
        drive.followTrajectory(linePath.build());
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