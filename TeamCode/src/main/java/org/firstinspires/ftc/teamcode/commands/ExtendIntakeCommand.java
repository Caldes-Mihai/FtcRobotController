package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ExtendIntakeCommand extends CommandBase {
    private final IntakeSubsystem subsystem;

    public ExtendIntakeCommand(IntakeSubsystem intakeSubsystem) {
        subsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
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