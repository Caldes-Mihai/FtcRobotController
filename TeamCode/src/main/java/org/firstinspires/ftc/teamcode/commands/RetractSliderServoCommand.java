package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SliderSubsystem;

public class RetractSliderServoCommand extends CommandBase {
    private final SliderSubsystem subsystem;

    public RetractSliderServoCommand(SliderSubsystem sliderSubsystem) {
        subsystem = sliderSubsystem;
        addRequirements(sliderSubsystem);
    }

    @Override
    public void execute() {
        subsystem.retractServo();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isRetracted();
    }
}