package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;

public class ResetDroneCommand extends CommandBase {
    private final DroneSubsystem subsystem;

    public ResetDroneCommand(DroneSubsystem droneSubsystem) {
        subsystem = droneSubsystem;
        addRequirements(droneSubsystem);
    }

    @Override
    public void execute() {
        subsystem.reset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}