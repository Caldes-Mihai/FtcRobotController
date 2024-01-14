package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class ActivateHolderCommand extends CommandBase {
    private final OuttakeSubsystem subsystem;

    public ActivateHolderCommand(OuttakeSubsystem outtakeSubsystem, boolean reversed) {
        subsystem = outtakeSubsystem;
        subsystem.setReversed(reversed);
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.activateHolder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}