package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Up Auto Op Mode", group = "Drive")
public class RedUpAutoOpMode extends CommandOpMode {

    private final String currentSpawnPosition = "up";
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
