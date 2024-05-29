package org.firstinspires.ftc.teamcode.mainopmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.handlers.HandleAuto;

@Autonomous(name = "Blue Up Auto Op Mode", group = "Drive")
public class BlueUpAutoOpMode extends CommandOpMode {

    private final HandleAuto.Positions currentSpawnPosition = HandleAuto.Positions.UP;
    private final boolean isRed = false;

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
