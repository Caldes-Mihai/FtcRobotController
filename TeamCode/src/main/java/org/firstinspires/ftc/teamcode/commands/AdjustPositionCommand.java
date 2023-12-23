package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AdjustPositionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class AdjustPositionCommand extends CommandBase {
    private final AdjustPositionSubsystem subsystem;
    private final MecanumDriveSubsystem driveSubsystem;

    public AdjustPositionCommand(AdjustPositionSubsystem adjustPositionSubsystem, MecanumDriveSubsystem
            driveSubsystem) {
        subsystem = adjustPositionSubsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(adjustPositionSubsystem);
    }

    @Override
    public void execute() {
        if (!driveSubsystem.isBusy()) subsystem.adjustPosition();
    }
}