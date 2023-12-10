package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Red Down Auto Op Mode")
public class RedDownAutoOpMode extends CommandOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private String currentSpawnPosition = "down";
    private boolean isRed = true;
    @Override
    public void initialize() {
        HandleAuto.init(isRed, currentSpawnPosition, this);
    }
}
