package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AlignWristVerticallyCommand extends CommandBase {
    private final WristSubsystem subsystem;

    public AlignWristVerticallyCommand(WristSubsystem wristSubsystem) {
        subsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        subsystem.vertical();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}