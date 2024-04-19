package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class PrepareOuttakeCommand extends SequentialCommandGroup {
    public PrepareOuttakeCommand(OuttakeSubsystem subsystem, AutoDriveSubsystem drive) {
        addCommands(
                new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 12),
                new ExtendSlidersCommand(subsystem),
                new StandBySlidersCommand(subsystem)
        );
        addRequirements(subsystem);
    }
}