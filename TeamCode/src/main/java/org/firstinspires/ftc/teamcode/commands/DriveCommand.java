package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TeleOpDriveSubsystem;

public class DriveCommand extends CommandBase {
    private final TeleOpDriveSubsystem subsystem;

    public DriveCommand(TeleOpDriveSubsystem teleOpDriveSubsystem) {
        subsystem = teleOpDriveSubsystem;
        addRequirements(teleOpDriveSubsystem);
    }

    @Override
    public void execute() {
        subsystem.drive();
    }
}