package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystems.AdjustPositionSubsystem;

public class PlaceCommand extends CommandBase {
    //implement arm/slider subsystem
    public PlaceCommand() {

    }
    @Override
    public boolean isFinished() {
        return true;
    }
}