package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class PickupCommand extends SequentialCommandGroup {
    public PickupCommand(IntakeSubsystem subsystem, OuttakeSubsystem outtakeSubsystem) {
        addCommands(
                new ActivateIntakeCommand(subsystem, false, false),
                new ActivateHolderCommand(outtakeSubsystem, false),
                new WaitUntilCommand(() -> subsystem.pixels == 2),
                new ResetPixelCount(subsystem),
                new ActivateIntakeCommand(subsystem, true, false),
                new DeactivateHolderCommand(outtakeSubsystem),
                new WaitCommand(500),
                new DeactivateIntakeCommand(subsystem)
        );
        addRequirements(subsystem, outtakeSubsystem);
    }
}