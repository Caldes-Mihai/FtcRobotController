package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class DeactivateClawCommand extends CommandBase {
    private final WristSubsystem subsystem;

    public DeactivateClawCommand(WristSubsystem sliderSubsystem) {
        subsystem = sliderSubsystem;
        addRequirements(sliderSubsystem);
    }

    @Override
    public void execute() {
        subsystem.release();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}