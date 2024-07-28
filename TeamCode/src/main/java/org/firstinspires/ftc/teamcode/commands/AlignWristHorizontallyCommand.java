package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AlignWristHorizontallyCommand extends CommandBase {
    private final WristSubsystem subsystem;

    public AlignWristHorizontallyCommand(WristSubsystem wristSubsystem) {
        subsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        subsystem.horizontal();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}