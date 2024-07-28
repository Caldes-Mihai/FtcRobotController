package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class ExtendOuttakeCommand extends SequentialCommandGroup {
    public ExtendOuttakeCommand(SliderSubsystem subsystem, WristSubsystem wristSubsystem) {
        addCommands(
                new DeactivateWristCommand(wristSubsystem),
                new AlignSlidersWithBoardCommand(subsystem),
                new WaitCommand(200),
                new ActivateWristCommand(wristSubsystem),
                new AlignWristWithBoardCommand(wristSubsystem)
        );
        addRequirements(subsystem, wristSubsystem);
    }
}