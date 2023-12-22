package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ActivateIntakeCommand extends CommandBase {
    private final IntakeSubsystem subsystem;

    public ActivateIntakeCommand(IntakeSubsystem intakeSubsystem, boolean reversed) {
        subsystem = intakeSubsystem;
        subsystem.setReversed(reversed);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.activate();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}