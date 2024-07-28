package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class PrepareOuttakeCommand extends SequentialCommandGroup {
    public PrepareOuttakeCommand(SliderSubsystem subsystem, WristSubsystem wristSubsystem, AutoDriveSubsystem drive) {
        addCommands(
                new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 12),
                new ExtendOuttakeCommand(subsystem.wristSubsystem)
        );
        addRequirements(subsystem);
    }
}