package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class ActivateClawCommand extends CommandBase {
    private final WristSubsystem subsystem;

    public ActivateClawCommand(WristSubsystem sliderSubsystem) {
        subsystem = sliderSubsystem;
        addRequirements(sliderSubsystem);
    }

    @Override
    public void execute() {
        subsystem.hold();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}