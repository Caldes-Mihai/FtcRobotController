package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Up Auto Op Mode", group = "Drive")
public class BlueUpAutoOpMode extends CommandOpMode {

    private final String currentSpawnPosition = "up";
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
