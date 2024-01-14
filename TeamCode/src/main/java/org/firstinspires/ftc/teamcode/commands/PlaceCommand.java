package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class PlaceCommand extends SequentialCommandGroup {
    public PlaceCommand(OuttakeSubsystem subsystem) {
        addCommands(
                new ActivateHolderCommand(subsystem, true),
                new WaitCommand(500),
                new DeactivateHolderCommand(subsystem)
        );
        addRequirements(subsystem);
    }
}