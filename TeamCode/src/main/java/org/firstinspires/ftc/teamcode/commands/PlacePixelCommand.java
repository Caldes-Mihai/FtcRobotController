package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class PlacePixelCommand extends SequentialCommandGroup {
    public PlacePixelCommand(IntakeSubsystem subsystem) {
        addCommands(
                new ActivateIntakeCommand(subsystem, true, true),
                new WaitCommand(500),
                new DeactivateIntakeCommand(subsystem)
        );
        addRequirements(subsystem);
    }
}