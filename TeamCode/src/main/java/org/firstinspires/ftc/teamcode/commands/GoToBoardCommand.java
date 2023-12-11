
package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToBoardCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final PropProcessor.Positions propPosition;
    private final boolean isRed;
    public GoToBoardCommand(MecanumDriveSubsystem drive, PropProcessor.Positions propPosition, boolean isRed) {
        this.drive = drive;
        this.propPosition = propPosition;
        this.isRed = isRed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        TrajectorySequenceBuilder destinationPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(45, isRed ? -36 : 36));
        if (propPosition.equals(PropProcessor.Positions.LEFT)) {
            destinationPath.strafeLeft(5);
        } else if (propPosition.equals(PropProcessor.Positions.CENTER)) {
            destinationPath.strafeRight(5);
        }
        drive.followTrajectory(destinationPath.build());
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