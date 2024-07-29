package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class RetractOuttakeCommand extends SequentialCommandGroup {
    public RetractOuttakeCommand(SliderSubsystem subsystem, WristSubsystem wristSubsystem) {
        addCommands(
                new RetractSliderServoCommand(subsystem),
                new WristPickupCommand(wristSubsystem)
        );
        addRequirements(subsystem, wristSubsystem);
    }
}