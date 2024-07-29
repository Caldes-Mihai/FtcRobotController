package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class OuttakePickupCommand extends SequentialCommandGroup {
    public OuttakePickupCommand(SliderSubsystem subsystem, WristSubsystem wristSubsystem) {
        addCommands(
                new SliderPickupCommand(subsystem),
                new WaitCommand(300),
                new ActivateClawCommand(wristSubsystem)
        );
        addRequirements(subsystem, wristSubsystem);
    }
}