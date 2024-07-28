package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class ActivateWristCommand extends CommandBase {
    private final WristSubsystem subsystem;

    public ActivateWristCommand(WristSubsystem wristSubsystem) {
        subsystem = wristSubsystem;
        addRequirements(wristSubsystem);
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