
package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToPixelStackCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final boolean isRed;
    private double stackY;
    public GoToPixelStackCommand(MecanumDriveSubsystem drive, boolean isRed) {
        this.drive = drive;
        this.isRed = isRed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        if (isRed)
            stackY = -36;
        else
            stackY = 36;
        TrajectorySequenceBuilder linePath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(drive.getPoseEstimate().getX(), stackY))
                .lineTo(new Vector2d(-57, stackY));

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