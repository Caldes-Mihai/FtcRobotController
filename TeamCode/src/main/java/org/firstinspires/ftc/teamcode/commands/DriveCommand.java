package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDriveSubsystem;

public class DriveCommand extends CommandBase {
    private final FieldCentricDriveSubsystem subsystem;

    public DriveCommand(FieldCentricDriveSubsystem fieldCentricDriveSubsystem) {
        subsystem = fieldCentricDriveSubsystem;
        addRequirements(fieldCentricDriveSubsystem);
    }

    @Override
    public void execute() {
        subsystem.drive();
    }
}