package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AlignWristToRightCommand extends CommandBase {
    private final WristSubsystem subsystem;

    public AlignWristToRightCommand(WristSubsystem wristSubsystem) {
        subsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        subsystem.rightDiagonal();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}