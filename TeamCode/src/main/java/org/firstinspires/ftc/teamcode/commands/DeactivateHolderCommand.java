package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class DeactivateHolderCommand extends CommandBase {
    private final OuttakeSubsystem subsystem;

    public DeactivateHolderCommand(OuttakeSubsystem outtakeSubsystem) {
        subsystem = outtakeSubsystem;
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.deactivateHolder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}