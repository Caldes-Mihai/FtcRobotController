package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SliderSubsystem;

public class ExtendSlidersCommand extends CommandBase {
    private final SliderSubsystem subsystem;
    private final double power;

    public ExtendSlidersCommand(SliderSubsystem sliderSubsystem, double p) {
        subsystem = sliderSubsystem;
        power = p;
        addRequirements(sliderSubsystem);
    }

    @Override
    public void execute() {
        subsystem.extend(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}