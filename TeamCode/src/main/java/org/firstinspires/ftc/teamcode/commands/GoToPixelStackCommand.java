
package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToPixelStackCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final PropProcessor processor;
    private final boolean isRed;
    private final boolean shouldTurn;
    private double stackY;
    private double degrees;
    private PropProcessor.Positions propPosition;
    public GoToPixelStackCommand(MecanumDriveSubsystem drive, PropProcessor processor, boolean isRed, boolean shouldTurn) {
        this.drive = drive;
        this.processor = processor;
        this.isRed = isRed;
        this.shouldTurn = shouldTurn;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        propPosition = processor.getPropPosition();
        if (isRed)
            stackY = -12;
        else
            stackY = 12;
        if (propPosition.equals(PropProcessor.Positions.LEFT)) {
            degrees = isRed ? 0 : Math.toRadians(180);
        } else if (propPosition.equals(PropProcessor.Positions.CENTER)) {
            degrees = isRed ? Math.toRadians(-90) : Math.toRadians(90);
        } else {
            degrees = isRed ? Math.toRadians(180) : 0;
        }
        TrajectorySequenceBuilder linePath = drive.trajectorySequenceBuilder(drive.getPoseEstimate());
        if(!shouldTurn)
            linePath.lineTo(new Vector2d(36, stackY))
                    .lineTo(new Vector2d(-57, stackY));
        else if(degrees != 0)
            linePath.lineToLinearHeading(new Pose2d(-57, stackY, drive.getPoseEstimate().getHeading() + degrees));
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