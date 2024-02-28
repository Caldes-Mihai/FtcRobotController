package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ResetPixelCount extends CommandBase {
    private final IntakeSubsystem subsystem;

    public ResetPixelCount(IntakeSubsystem intakeSubsystem) {
        subsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.resetPixels();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}