package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class PrepareOuttake extends SequentialCommandGroup {
    public PrepareOuttake(OuttakeSubsystem subsystem, MecanumDriveSubsystem drive) {
        addCommands(
                new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 12),
                new ExtendSlidersCommand(subsystem)
        );
        addRequirements(subsystem);
    }
}