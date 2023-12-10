package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AdjustPositionSubsystem;

public class AdjustPositionCommand extends CommandBase {
    private AdjustPositionSubsystem subsystem;
    public AdjustPositionCommand(AdjustPositionSubsystem adjustPositionSubsystem) {
        subsystem = adjustPositionSubsystem;
        addRequirements(adjustPositionSubsystem);
    }

    @Override
    public void execute() {
        subsystem.adjustPosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}