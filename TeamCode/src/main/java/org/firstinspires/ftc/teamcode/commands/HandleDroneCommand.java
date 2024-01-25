package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;

public class HandleDroneCommand extends CommandBase {
    private final DroneSubsystem subsystem;

    public HandleDroneCommand(DroneSubsystem droneSubsystem) {
        subsystem = droneSubsystem;
        addRequirements(droneSubsystem);
    }

    @Override
    public void execute() {
        subsystem.handle();
    }
}