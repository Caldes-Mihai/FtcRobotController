package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class RetractOuttakeCommand extends SequentialCommandGroup {
    public RetractOuttakeCommand(SliderSubsystem subsystem, WristSubsystem wristSubsystem) {
        addCommands(
                new RetractSliderServoCommand(subsystem),
                new WaitCommand(500),
                new WristPickupCommand(wristSubsystem),
                new DeactivateWristCommand(wristSubsystem)
        );
        addRequirements(subsystem, wristSubsystem);
    }
}