package org.firstinspires.ftc.teamcode.mainopmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.handlers.HandleAuto;

@Autonomous(name = "Red Up Auto Op Mode", group = "Drive")
public class RedUpAutoOpMode extends CommandOpMode {

    private final HandleAuto.Positions currentSpawnPosition = HandleAuto.Positions.UP;
    private final boolean isRed = true;

    @Override
    public void initialize() {
        HandleAuto.init(isRed, currentSpawnPosition, this);
    }

    @Override
    public void run() {
        HandleAuto.run();
        super.run();
    }
}
