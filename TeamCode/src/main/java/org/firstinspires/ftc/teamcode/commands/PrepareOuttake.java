package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class PrepareOuttake extends SequentialCommandGroup {
    public PrepareOuttake(OuttakeSubsystem subsystem, AutoDriveSubsystem drive) {
        addCommands(
                new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 12),
                new ExtendSlidersCommand(subsystem)
        );
        addRequirements(subsystem);
    }
}