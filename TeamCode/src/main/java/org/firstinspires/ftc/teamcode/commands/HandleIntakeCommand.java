package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class HandleIntakeCommand extends CommandBase {
    private IntakeSubsystem subsystem;
    public HandleIntakeCommand(IntakeSubsystem intakeSubsystem) {
        subsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.handle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}