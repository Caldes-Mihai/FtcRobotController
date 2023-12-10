package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Up Auto Op Mode")
public class RedUpAutoOpMode extends CommandOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private String currentSpawnPosition = "up";
    private boolean isRed = true;
    @Override
    public void initialize() {
        HandleAuto.init(isRed, currentSpawnPosition, this);
    }
}
