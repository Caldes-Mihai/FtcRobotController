package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class WristPickupCommand extends CommandBase {
    private final WristSubsystem subsystem;

    public WristPickupCommand(WristSubsystem wristSubsystem) {
        subsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        subsystem.pickup();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}