package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class ReleaseClaw2Command extends CommandBase {
    private final WristSubsystem subsystem;

    public ReleaseClaw2Command(WristSubsystem wristSubsystem) {
        subsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        subsystem.releaseClaw2();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}