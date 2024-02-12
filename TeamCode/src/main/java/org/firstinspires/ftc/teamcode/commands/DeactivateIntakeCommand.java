package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class DeactivateIntakeCommand extends CommandBase {
    private final IntakeSubsystem subsystem;

    public DeactivateIntakeCommand(IntakeSubsystem intakeSubsystem) {
        subsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.deactivate();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}