package org.firstinspires.ftc.teamcode.handlers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.cache.CacheManager;
import org.firstinspires.ftc.teamcode.commands.AdjustPositionCommand;
import org.firstinspires.ftc.teamcode.commands.DeactivateClawCommand;
import org.firstinspires.ftc.teamcode.commands.GoFromBoardToPixelStackCommand;
import org.firstinspires.ftc.teamcode.commands.GoToBoardCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPixelStackCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPropCommand;
import org.firstinspires.ftc.teamcode.commands.PickupCommand;
import org.firstinspires.ftc.teamcode.commands.PlacePixelCommand;
import org.firstinspires.ftc.teamcode.commands.PrepareOuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.RetractSlidersCommand;
import org.firstinspires.ftc.teamcode.commands.StandBySlidersCommand;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Config
public class HandleAuto {

    public static long delta;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private static PropProcessor processor;
    private static AprilTagProcessor aprilTagProcessor;
    /**
     * The variable to store our instance of the vision portal.
     */
    private static VisionPortal visionPortal;
    private static AutoDriveSubsystem autoDriveSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static OuttakeSubsystem outtakeSubsystem;
    private static PropProcessor.Positions propPosition;
    private static CommandOpMode opMode;
    private static HardwareMap hardwareMap;
    private static Telemetry telemetry;
    private static CacheManager cacheManager;
    private static long currentTime;
    private static long lastTime;

    public static void init(boolean isRed, Positions currentSpawnPosition, CommandOpMode op) {
        opMode = op;
        hardwareMap = opMode.hardwareMap;
        telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        opMode.telemetry = telemetry;
        cacheManager = new CacheManager(hardwareMap);
        processor = new PropProcessor(telemetry);
        processor.setRed(isRed);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .build();
        aprilTagProcessor.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "0"))
                .addProcessors(processor, aprilTagProcessor)
                .build();
        setManualExposure(6, 250);
        autoDriveSubsystem = new AutoDriveSubsystem(hardwareMap, aprilTagProcessor, currentSpawnPosition, isRed);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, null, null);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap, intakeSubsystem, null);
        opMode.register(autoDriveSubsystem, intakeSubsystem, outtakeSubsystem);

        if (currentSpawnPosition.equals(Positions.DOWN))
            opMode.schedule(new SequentialCommandGroup(
                    new GoToPropCommand(autoDriveSubsystem, processor, isRed, false),
                    new PlacePixelCommand(intakeSubsystem),
                    new AdjustPositionCommand(autoDriveSubsystem),
                    new GoToPixelStackCommand(autoDriveSubsystem, isRed, false),
                    new PickupCommand(intakeSubsystem, outtakeSubsystem),
                    new AdjustPositionCommand(autoDriveSubsystem),
                    new ParallelCommandGroup(
                            new GoToBoardCommand(autoDriveSubsystem, isRed, false),
                            new PrepareOuttakeCommand(outtakeSubsystem, autoDriveSubsystem)),
                    new DeactivateClawCommand(outtakeSubsystem),
                    new AdjustPositionCommand(autoDriveSubsystem),
                    new ParallelCommandGroup(
                            new GoFromBoardToPixelStackCommand(autoDriveSubsystem, isRed, false),
                            new RetractSlidersCommand(outtakeSubsystem),
                            new StandBySlidersCommand(outtakeSubsystem)),
                    new PickupCommand(intakeSubsystem, outtakeSubsystem),
                    new AdjustPositionCommand(autoDriveSubsystem),
                    new ParallelCommandGroup(
                            new GoToBoardCommand(autoDriveSubsystem, isRed, false),
                            new PrepareOuttakeCommand(outtakeSubsystem, autoDriveSubsystem)),
                    new DeactivateClawCommand(outtakeSubsystem),
                    new RetractSlidersCommand(outtakeSubsystem),
                    new StandBySlidersCommand(outtakeSubsystem),
                    new AdjustPositionCommand(autoDriveSubsystem)
            ));
        else
            opMode.schedule(new SequentialCommandGroup(
                    new GoToPropCommand(autoDriveSubsystem, processor, isRed, true),
                    new PlacePixelCommand(intakeSubsystem),
                    new AdjustPositionCommand(autoDriveSubsystem),
                    new ParallelCommandGroup(
                            new GoToBoardCommand(autoDriveSubsystem, isRed, true),
                            new PrepareOuttakeCommand(outtakeSubsystem, autoDriveSubsystem)),
                    new DeactivateClawCommand(outtakeSubsystem),
                    new AdjustPositionCommand(autoDriveSubsystem),
                    new ParallelCommandGroup(
                            new GoFromBoardToPixelStackCommand(autoDriveSubsystem, isRed, true),
                            new RetractSlidersCommand(outtakeSubsystem),
                            new StandBySlidersCommand(outtakeSubsystem)),
                    new PickupCommand(intakeSubsystem, outtakeSubsystem),
                    new AdjustPositionCommand(autoDriveSubsystem),
                    new ParallelCommandGroup(
                            new GoToBoardCommand(autoDriveSubsystem, isRed, true),
                            new PrepareOuttakeCommand(outtakeSubsystem, autoDriveSubsystem)),
                    new DeactivateClawCommand(outtakeSubsystem),
                    new AdjustPositionCommand(autoDriveSubsystem),
                    new ParallelCommandGroup(
                            new GoFromBoardToPixelStackCommand(autoDriveSubsystem, isRed, true),
                            new RetractSlidersCommand(outtakeSubsystem),
                            new StandBySlidersCommand(outtakeSubsystem)),
                    new PickupCommand(intakeSubsystem, outtakeSubsystem),
                    new AdjustPositionCommand(autoDriveSubsystem),
                    new ParallelCommandGroup(
                            new GoToBoardCommand(autoDriveSubsystem, isRed, true),
                            new PrepareOuttakeCommand(outtakeSubsystem, autoDriveSubsystem)),
                    new DeactivateClawCommand(outtakeSubsystem),
                    new RetractSlidersCommand(outtakeSubsystem),
                    new StandBySlidersCommand(outtakeSubsystem),
                    new AdjustPositionCommand(autoDriveSubsystem)
            ));
    }

    private static void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!opMode.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                opMode.sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            opMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            opMode.sleep(20);
        }
    }

    public static PropProcessor.Positions getPropPosition() {
        return propPosition;
    }

    public static void setPropPosition(PropProcessor.Positions position) {
        propPosition = position;
        visionPortal.setProcessorEnabled(processor, false);
    }

    public static void run() {
        cacheManager.clear();
        currentTime = System.currentTimeMillis();
        delta = currentTime - lastTime;
        lastTime = currentTime;
        telemetry.addData("time", 1000 / delta + "hz");
        telemetry.update();
    }

    public enum Trajectories {
        GO_TO_BOARD,
        GO_TO_PIXEL_STACK,
        GO_TO_PROP,
        GO_FROM_BOARD_TO_PIXEL_STACK
    }

    public enum Positions {
        DOWN,
        UP
    }
}
