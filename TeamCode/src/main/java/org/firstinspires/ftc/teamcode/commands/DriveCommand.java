package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private DriveSubsystem subsystem;
    public DriveCommand(DriveSubsystem driveSubsystem) {
        subsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        subsystem.drive();
    }
}