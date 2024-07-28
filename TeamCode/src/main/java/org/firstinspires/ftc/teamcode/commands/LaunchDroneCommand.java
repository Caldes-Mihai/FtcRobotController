package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;

public class LaunchDroneCommand extends CommandBase {
    private final DroneSubsystem subsystem;

    public LaunchDroneCommand(DroneSubsystem droneSubsystem) {
        subsystem = droneSubsystem;
        addRequirements(droneSubsystem);
    }

    @Override
    public void execute() {
        subsystem.launch();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}