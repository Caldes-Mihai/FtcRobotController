package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class DeactivateWristCommand extends CommandBase {
    private final WristSubsystem subsystem;

    public DeactivateWristCommand(WristSubsystem wristSubsystem) {
        subsystem = wristSubsystem;
        addRequirements(wristSubsystem);
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