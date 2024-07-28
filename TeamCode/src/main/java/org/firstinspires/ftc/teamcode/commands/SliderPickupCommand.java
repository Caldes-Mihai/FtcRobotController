package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SliderSubsystem;

public class SliderPickupCommand extends CommandBase {
    private final SliderSubsystem subsystem;

    public SliderPickupCommand(SliderSubsystem sliderSubsystem) {
        subsystem = sliderSubsystem;
        addRequirements(sliderSubsystem);
    }

    @Override
    public void execute() {
        subsystem.pickupPixel();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isExtended();
    }
}