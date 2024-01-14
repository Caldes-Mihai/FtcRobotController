package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class PickupCommand extends SequentialCommandGroup {
    public PickupCommand(IntakeSubsystem subsystem, OuttakeSubsystem outtakeSubsystem) {
        addCommands(
                new ActivateIntakeCommand(subsystem, false),
                new ActivateHolderCommand(outtakeSubsystem, false),
                new WaitCommand(500),
                new DeactivateIntakeCommand(subsystem),
                new DeactivateHolderCommand(outtakeSubsystem)
        );
        addRequirements(subsystem, outtakeSubsystem);
    }
}