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
                new WaitUntilCommand(() -> subsystem.pixel1 && subsystem.pixel2),
                new ActivateIntakeCommand(subsystem, true, false),
                new WaitCommand(500),
                new DeactivateIntakeCommand(subsystem)
        );
        addRequirements(subsystem, outtakeSubsystem);
    }
}