package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.HandleAuto;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


public class GoToBoardCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final boolean isRed;
    private final boolean isFirstTime;
    private final boolean shouldTurn;
    private double strafe;
    private double degrees;
    private PropProcessor.Positions propPosition;

    public GoToBoardCommand(MecanumDriveSubsystem drive, boolean isRed, boolean isFirstTime, boolean shouldTurn) {
        this.drive = drive;
        this.isRed = isRed;
        this.isFirstTime = isFirstTime;
        this.shouldTurn = shouldTurn;
        addRequirements(drive);
    }

    public GoToBoardCommand(MecanumDriveSubsystem drive, boolean isRed, boolean isFirstTime) {
        this(drive, isRed, isFirstTime, false);
    }

    @Override
    public void initialize() {
        propPosition = HandleAuto.getPropPosition();
        if (propPosition.equals(PropProcessor.Positions.LEFT)) {
            strafe = 5;
            degrees = isRed ? 0 : Math.toRadians(180);
        } else if (propPosition.equals(PropProcessor.Positions.CENTER)) {
            strafe = 0;
            degrees = isRed ? Math.toRadians(-90) : Math.toRadians(90);
        } else {
            strafe = -5;
            degrees = isRed ? Math.toRadians(180) : 0;
        }
        TrajectorySequenceBuilder destinationPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate());
        if (isFirstTime && !shouldTurn)
            destinationPath.lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineTo(new Vector2d(36, isRed ? -36 + strafe : 36 + strafe));
        else if (shouldTurn)
            destinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12))
                    .lineToLinearHeading(new Pose2d(36, isRed ? -36 + strafe : 36 + strafe, drive.getPoseEstimate().getHeading() + degrees));
        else
            destinationPath
                    .lineTo(new Vector2d(36, isRed ? -12 : 12));
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