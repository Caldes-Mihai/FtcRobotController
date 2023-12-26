package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.cache.CacheManager;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;
import org.firstinspires.ftc.teamcode.commands.AdjustPositionCommand;
import org.firstinspires.ftc.teamcode.commands.GoToBoardCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPixelStackCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPropCommand;
import org.firstinspires.ftc.teamcode.commands.PickupCommand;
import org.firstinspires.ftc.teamcode.commands.PlaceCommand;
import org.firstinspires.ftc.teamcode.commands.PlacePixelCommand;
import org.firstinspires.ftc.teamcode.commands.PrepareOuttake;
import org.firstinspires.ftc.teamcode.commands.ResetHolderCommand;
import org.firstinspires.ftc.teamcode.commands.RetractSlidersCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class HandleAuto {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private static PropProcessor processor;
    private static AprilTagProcessor aprilTagProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private static VisionPortal visionPortal;
    private static SampleMecanumDrive drive;
    private static AutoDriveSubsystem autoDriveSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static OuttakeSubsystem outtakeSubsystem;
    private static Pose2d startPose;
    private static CacheableMotor intake;
    private static CacheableServo sliders;
    private static CacheableServo holder;
    private static PropProcessor.Positions propPosition;
    private static CommandOpMode opMode;
    private static HardwareMap hardwareMap;
    private static Telemetry telemetry;
    private static CacheManager cacheManager;
    private static long currentTime;
    private static long lastTime;
    private static long delta;

    public static void init(boolean isRed, String currentSpawnPosition, CommandOpMode op) {
        opMode = op;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        cacheManager = new CacheManager(hardwareMap);
        intake = new CacheableMotor(hardwareMap, "intake");
        sliders = new CacheableServo(hardwareMap, "sliders", 0, 360);
        holder = new CacheableServo(hardwareMap, "holder", 0, 360);
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
        drive = new SampleMecanumDrive(hardwareMap);
        autoDriveSubsystem = new AutoDriveSubsystem(drive, aprilTagProcessor, false);
        intakeSubsystem = new IntakeSubsystem(intake);
        outtakeSubsystem = new OuttakeSubsystem(sliders, holder);
        opMode.register(autoDriveSubsystem, intakeSubsystem, outtakeSubsystem);
        if (!isRed) {
            if (currentSpawnPosition.equals("down"))
                startPose = new Pose2d(-36, 63, Math.toRadians(90));
            else
                startPose = new Pose2d(12, 63, Math.toRadians(90));
        } else {
            if (currentSpawnPosition.equals("down"))
                startPose = new Pose2d(-36, -63, Math.toRadians(-90));
            else
                startPose = new Pose2d(12, -63, Math.toRadians(-90));
        }
        drive.setPoseEstimate(startPose);
        //if(currentSpawnPosition.equals("down"))
        opMode.schedule(new SequentialCommandGroup(
                new GoToPropCommand(autoDriveSubsystem, processor, isRed),
                new PlacePixelCommand(intakeSubsystem),
                new AdjustPositionCommand(autoDriveSubsystem),
                new GoToPixelStackCommand(autoDriveSubsystem, isRed, true),
                new PickupCommand(intakeSubsystem),
                new AdjustPositionCommand(autoDriveSubsystem),
                new ParallelCommandGroup(
                        new GoToBoardCommand(autoDriveSubsystem, isRed, true),
                        new PrepareOuttake(outtakeSubsystem, autoDriveSubsystem)),
                new PlaceCommand(outtakeSubsystem),
                new AdjustPositionCommand(autoDriveSubsystem),
                new ParallelCommandGroup(
                        new GoToPixelStackCommand(autoDriveSubsystem, isRed, false),
                        new ResetHolderCommand(outtakeSubsystem),
                        new RetractSlidersCommand(outtakeSubsystem)),
                new PickupCommand(intakeSubsystem),
                new AdjustPositionCommand(autoDriveSubsystem),
                new ParallelCommandGroup(
                        new GoToBoardCommand(autoDriveSubsystem, isRed, true),
                        new PrepareOuttake(outtakeSubsystem, autoDriveSubsystem)),
                new PlaceCommand(outtakeSubsystem),
                new RetractSlidersCommand(outtakeSubsystem),
                new AdjustPositionCommand(autoDriveSubsystem)
        ));
         /*else
            opMode.schedule(new SequentialCommandGroup(
                    new GoToPropCommand(mecanumDriveSubsystem, processor, isRed),
                    new PlacePixelCommand(intakeSubsystem),
                    new GoToBoardCommand(mecanumDriveSubsystem, processor, isRed, true, true),
                    new PlaceCommand(),
                    new GoToPixelStackCommand(mecanumDriveSubsystem, processor, isRed, false),
                    new PickupCommand(intakeSubsystem),
                    new GoToBoardCommand(mecanumDriveSubsystem, processor, isRed, true),
                    new PlaceCommand(),
                    new GoToPixelStackCommand(mecanumDriveSubsystem, processor, isRed, false),
                    new PickupCommand(intakeSubsystem),
                    new GoToBoardCommand(mecanumDriveSubsystem, processor, isRed, true),
                    new PlaceCommand()
            ));*/
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
    }

    public static void run() {
        cacheManager.clear();
        currentTime = System.currentTimeMillis();
        delta = currentTime - lastTime;
        lastTime = currentTime;
        telemetry.addData("time", 1000 / delta + "hz");
        telemetry.update();
    }
}
