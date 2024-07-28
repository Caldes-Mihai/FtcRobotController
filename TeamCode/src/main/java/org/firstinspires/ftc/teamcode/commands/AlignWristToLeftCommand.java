package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AlignWristToLeftCommand extends CommandBase {
    private final WristSubsystem subsystem;

    public AlignWristToLeftCommand(WristSubsystem wristSubsystem) {
        subsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        subsystem.leftDiagonal();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}