package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class ExtendSlidersCommand extends CommandBase {
    private final OuttakeSubsystem subsystem;

    public ExtendSlidersCommand(OuttakeSubsystem outtakeSubsystem) {
        subsystem = outtakeSubsystem;
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.extend();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}