package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class PlaceCommand extends CommandBase {
    private final OuttakeSubsystem subsystem;

    public PlaceCommand(OuttakeSubsystem outtakeSubsystem) {
        subsystem = outtakeSubsystem;
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.release();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isReleased();
    }
}