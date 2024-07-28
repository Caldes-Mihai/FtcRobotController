package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class ReleaseClaw1Command extends CommandBase {
    private final WristSubsystem subsystem;

    public ReleaseClaw1Command(WristSubsystem wristSubsystem) {
        subsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        subsystem.releaseClaw1();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}