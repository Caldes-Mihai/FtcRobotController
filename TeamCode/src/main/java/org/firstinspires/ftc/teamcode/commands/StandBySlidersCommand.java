package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class StandBySlidersCommand extends CommandBase {
    private final OuttakeSubsystem subsystem;

    public StandBySlidersCommand(OuttakeSubsystem outtakeSubsystem) {
        subsystem = outtakeSubsystem;
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.standBy();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}