
package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToBoardCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final PropProcessor processor;
    private final boolean isRed;
    private final boolean shouldTurn;
    private double strafe;
    private double degrees;
    private PropProcessor.Positions propPosition;
    public GoToBoardCommand(MecanumDriveSubsystem drive, PropProcessor processor, boolean isRed, boolean shouldTurn) {
        this.drive = drive;
        this.processor = processor;
        this.isRed = isRed;
        this.shouldTurn = shouldTurn;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        propPosition = processor.getPropPosition();
        if (propPosition.equals(PropProcessor.Positions.LEFT)) {
            strafe = 5;
            degrees = isRed ? 0 : Math.toRadians(180);
        } else if (propPosition.equals(PropProcessor.Positions.CENTER)) {
            strafe = 0;
            degrees = isRed ? Math.toRadians(90) : Math.toRadians(-90);
        } else {
            strafe = -5;
            degrees = isRed ? Math.toRadians(180) : 0;
        }
        TrajectorySequenceBuilder destinationPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate());
        if(degrees != 0 && shouldTurn)
            destinationPath.turn(degrees);
        destinationPath
                .lineTo(new Vector2d(drive.getPoseEstimate().getX(), isRed ? -36 : 36))
                .lineTo(new Vector2d(45, isRed ? -36 : 36));
        if(strafe != 0)
            destinationPath.lineTo(new Vector2d(45, isRed ? -36 + strafe : 36 + strafe));
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