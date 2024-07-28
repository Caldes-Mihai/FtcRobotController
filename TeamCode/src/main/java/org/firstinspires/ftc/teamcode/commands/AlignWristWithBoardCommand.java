package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AlignWristWithBoardCommand extends CommandBase {
    private final WristSubsystem subsystem;

    public AlignWristWithBoardCommand(WristSubsystem wristSubsystem) {
        subsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        subsystem.alignWithBoard();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}