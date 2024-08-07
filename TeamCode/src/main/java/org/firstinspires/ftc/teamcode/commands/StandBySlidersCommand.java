package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SliderSubsystem;

public class StandBySlidersCommand extends CommandBase {
    private final SliderSubsystem subsystem;

    public StandBySlidersCommand(SliderSubsystem sliderSubsystem) {
        subsystem = sliderSubsystem;
        addRequirements(sliderSubsystem);
    }

    @Override
    public void execute() {
        subsystem.standBy();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}