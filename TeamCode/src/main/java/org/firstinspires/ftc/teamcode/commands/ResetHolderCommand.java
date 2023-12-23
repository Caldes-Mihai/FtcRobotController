package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class ResetHolderCommand extends CommandBase {
    private final OuttakeSubsystem subsystem;

    public ResetHolderCommand(OuttakeSubsystem outtakeSubsystem) {
        subsystem = outtakeSubsystem;
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.resetHolder();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isReset();
    }
}