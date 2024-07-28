package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class RetractIntakeCommand extends CommandBase {
    private final IntakeSubsystem subsystem;

    public RetractIntakeCommand(IntakeSubsystem intakeSubsystem) {
        subsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}