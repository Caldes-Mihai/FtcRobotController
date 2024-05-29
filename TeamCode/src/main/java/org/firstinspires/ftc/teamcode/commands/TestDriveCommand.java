package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.RobotCentricDriveSubsystem;

public class TestDriveCommand extends CommandBase {
    private final RobotCentricDriveSubsystem subsystem;

    public TestDriveCommand(RobotCentricDriveSubsystem teleOpDriveSubsystem) {
        subsystem = teleOpDriveSubsystem;
        addRequirements(teleOpDriveSubsystem);
    }

    @Override
    public void execute() {
        subsystem.drive();
    }
}