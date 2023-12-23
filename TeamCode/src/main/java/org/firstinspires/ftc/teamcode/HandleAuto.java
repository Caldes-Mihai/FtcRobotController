package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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
import org.firstinspires.ftc.teamcode.subsystems.AdjustPositionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
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
    private static MecanumDriveSubsystem mecanumDriveSubsystem;
    private static AdjustPositionSubsystem adjustPositionSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static OuttakeSubsystem outtakeSubsystem;
    private static Pose2d startPose;
    private static MotorEx intake;
    private static ServoEx sliders;
    private static ServoEx holder;

    public static void init(boolean isRed, String currentSpawnPosition, CommandOpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        intake = new MotorEx(hardwareMap, "intake");
        sliders = new SimpleServo(hardwareMap, "sliders", 0, 360);
        sliders = new SimpleServo(hardwareMap, "holder", 0, 360);
        processor = new PropProcessor(opMode.telemetry);
        processor.setRed(isRed);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .build();
        aprilTagProcessor.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "0"))
                .addProcessors(processor, aprilTagProcessor)
                .build();
        setManualExposure(6, 250, opMode);
        drive = new SampleMecanumDrive(hardwareMap);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(drive, false);
        adjustPositionSubsystem = new AdjustPositionSubsystem(drive, aprilTagProcessor, isRed);
        intakeSubsystem = new IntakeSubsystem(intake);
        outtakeSubsystem = new OuttakeSubsystem(sliders, holder);
        opMode.register(mecanumDriveSubsystem, adjustPositionSubsystem, intakeSubsystem, outtakeSubsystem);
        adjustPositionSubsystem.setDefaultCommand(new AdjustPositionCommand(adjustPositionSubsystem, mecanumDriveSubsystem));
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
                new GoToPropCommand(mecanumDriveSubsystem, processor, isRed),
                new PlacePixelCommand(intakeSubsystem),
                new GoToPixelStackCommand(mecanumDriveSubsystem, processor, isRed, true),
                new PickupCommand(intakeSubsystem),
                new ParallelCommandGroup(
                        new GoToBoardCommand(mecanumDriveSubsystem, processor, isRed, true),
                        new PrepareOuttake(outtakeSubsystem, mecanumDriveSubsystem)),
                new PlaceCommand(outtakeSubsystem),
                new ParallelCommandGroup(
                        new GoToPixelStackCommand(mecanumDriveSubsystem, processor, isRed, false),
                        new ResetHolderCommand(outtakeSubsystem),
                        new RetractSlidersCommand(outtakeSubsystem)),
                new PickupCommand(intakeSubsystem),
                new ParallelCommandGroup(
                        new GoToBoardCommand(mecanumDriveSubsystem, processor, isRed, true),
                        new PrepareOuttake(outtakeSubsystem, mecanumDriveSubsystem)),
                new PlaceCommand(outtakeSubsystem),
                new RetractSlidersCommand(outtakeSubsystem),
                new RunCommand(() -> visionPortal.close())
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
                    new PlaceCommand(),
                    new RunCommand(() -> visionPortal.close())
            ));*/
    }

    private static void setManualExposure(int exposureMS, int gain, CommandOpMode opMode) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            opMode.telemetry.addData("Camera", "Waiting");
            opMode.telemetry.update();
            while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
            opMode.telemetry.addData("Camera", "Ready");
            opMode.telemetry.update();
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
}
