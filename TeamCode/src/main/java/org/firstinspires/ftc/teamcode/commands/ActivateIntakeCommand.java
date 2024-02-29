package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ActivateIntakeCommand extends CommandBase {
    private final IntakeSubsystem subsystem;
    private final boolean reversed;

    public ActivateIntakeCommand(IntakeSubsystem intakeSubsystem, boolean reversed, boolean slow) {
        subsystem = intakeSubsystem;
        subsystem.setReversed(reversed);
        subsystem.setSlow(slow);
        this.reversed = reversed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (!reversed) {
            subsystem.extend();
            subsystem.lower();
        }
        subsystem.activate();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}