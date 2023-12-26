package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;

public class AdjustPositionCommand extends CommandBase {
    private final AutoDriveSubsystem subsystem;

    public AdjustPositionCommand(AutoDriveSubsystem
                                         subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.adjustPosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}