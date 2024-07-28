package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SliderSubsystem;

public class AlignSlidersWithBoardCommand extends CommandBase {
    private final SliderSubsystem subsystem;

    public AlignSlidersWithBoardCommand(SliderSubsystem sliderSubsystem) {
        subsystem = sliderSubsystem;
        addRequirements(sliderSubsystem);
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