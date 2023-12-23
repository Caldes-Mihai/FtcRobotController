package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class RetractSlidersCommand extends CommandBase {
    private final OuttakeSubsystem subsystem;

    public RetractSlidersCommand(OuttakeSubsystem outtakeSubsystem) {
        subsystem = outtakeSubsystem;
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        subsystem.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}