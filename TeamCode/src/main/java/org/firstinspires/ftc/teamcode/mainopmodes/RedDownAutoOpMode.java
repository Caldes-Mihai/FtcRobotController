package org.firstinspires.ftc.teamcode.mainopmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.handlers.HandleAuto;

@Autonomous(name = "Red Down Auto Op Mode", group = "Drive")
public class RedDownAutoOpMode extends CommandOpMode {

    private final HandleAuto.Positions currentSpawnPosition = HandleAuto.Positions.DOWN;
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
