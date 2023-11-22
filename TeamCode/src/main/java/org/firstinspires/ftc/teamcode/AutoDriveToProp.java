package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.processor.PropProcessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Example VisionPortal OpMode")
public class AutoDriveToProp extends LinearOpMode {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private PropProcessor processor;
    private AprilTagProcessor aprilTagProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private List<String> spawnPositions = Arrays.asList("down", "up");
    private String currentSpawnPosition = spawnPositions.get(0);
    private boolean isRed = false;
    private boolean held = false;
    private double degrees = 0;
    int lineY = 0;
    int strafe = 0;
    @Override
    public void runOpMode() {
        processor = new PropProcessor(telemetry);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .build();
        processor.setRed(true);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "0"))
                .addProcessors(processor, aprilTagProcessor)
                .build();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        while(!opModeIsActive()) {
            int pos = spawnPositions.indexOf(currentSpawnPosition);
            if(gamepad1.dpad_left) {
                if(pos != 0 && !held)
                    currentSpawnPosition = spawnPositions.get(pos - 1);
                held = true;
            } else if(gamepad1.dpad_right) {
                if(pos != spawnPositions.size() - 1 && !held)
                    currentSpawnPosition = spawnPositions.get(pos + 1);
                held = true;
            } else if(gamepad1.a) {
                if(!held)
                    isRed = !isRed;
                held = true;
            } else {
                held = false;
            }
            telemetry.addData("POSITION", currentSpawnPosition);
            telemetry.addData("TEAM", isRed ? "RED" : "BLUE");
            telemetry.update();
        }
        waitForStart();
        Pose2d startPose = null;
        if(!isRed) {
            if(currentSpawnPosition.equals("down"))
                startPose = new Pose2d(-36, 63, Math.toRadians(-90));
            else
                startPose = new Pose2d(12, 63, Math.toRadians(-90));
        } else {
            if(currentSpawnPosition.equals("down"))
                startPose = new Pose2d(-36, -63, Math.toRadians(90));
            else
                startPose = new Pose2d(12, -63, Math.toRadians(90));
        }
        drive.setPoseEstimate(startPose);
        adjustPosition(drive);
        TrajectorySequenceBuilder propPath = drive.trajectorySequenceBuilder(startPose)
                .forward(27);
        PropProcessor.Positions propPosition = processor.getPropPosition();
        if(isRed)
            lineY = -36;
        else
            lineY = 36;
        if (propPosition.equals(PropProcessor.Positions.LEFT)) {
            degrees = Math.toRadians(90);
            strafe = 5;
        }
        else if (propPosition.equals(PropProcessor.Positions.CENTER)) {
            degrees = Math.toRadians(-90);
            strafe = -5;
        }
        else {
            if(isRed) propPath.strafeLeft(5);
            else propPath.strafeRight(5);
            strafe = 0;
        }
        propPath.turn(degrees);
        drive.followTrajectorySequence(propPath.build());
        place();
        if(propPosition.equals(PropProcessor.Positions.LEFT))
            drive.turn(isRed ? 0 : -2 * degrees);
        else if(propPosition.equals(PropProcessor.Positions.CENTER))
            drive.turn(Math.toRadians(isRed ? 90 : -90));
        else {
            drive.turn(isRed ? -2 * degrees : 0);
        }
        //get pixel then place
        TrajectorySequenceBuilder linePath = drive.trajectorySequenceBuilder(drive.getPoseEstimate());
        linePath.lineTo(new Vector2d(-54, lineY))
                .turn(Math.toRadians(180));
        drive.followTrajectorySequence(linePath.build());
        pickup();
        TrajectorySequenceBuilder destinationPath = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(45, isRed ? -36 : 36));
        if(strafe > 0)
            destinationPath.strafeLeft(strafe);
        else if(strafe < 0)
            destinationPath.strafeRight(strafe);
        drive.followTrajectorySequence(destinationPath.build());
        place();
        visionPortal.close();
    }
    public void adjustPosition(SampleMecanumDrive drive) {
        ArrayList<AprilTagDetection> aprilTags = aprilTagProcessor.getDetections();
        if(aprilTags.size() == 0) {
            adjustPosition(drive);
            return;
        }
        //calculate for apriltags (test april tag angle)
        AprilTagDetection aprilTag = aprilTags.get(0);
        if(aprilTag.id == 10) {
            drive.setPoseEstimate(new Pose2d(aprilTag.ftcPose.x - 72, aprilTag.ftcPose.x + 42, aprilTag.ftcPose.yaw));
        } else if(aprilTag.id == 7) {
            drive.setPoseEstimate(new Pose2d(aprilTag.ftcPose.x - 72, aprilTag.ftcPose.x - 42, aprilTag.ftcPose.yaw));
        } else if(aprilTag.id == 1) {
            drive.setPoseEstimate(new Pose2d(aprilTag.ftcPose.x + 61, aprilTag.ftcPose.x + 42, aprilTag.ftcPose.yaw));
        } else if(aprilTag.id == 2) {
            drive.setPoseEstimate(new Pose2d(aprilTag.ftcPose.x + 61, aprilTag.ftcPose.x + 36, aprilTag.ftcPose.yaw));
        } else if(aprilTag.id == 3) {
            drive.setPoseEstimate(new Pose2d(aprilTag.ftcPose.x + 61, aprilTag.ftcPose.x + 30, aprilTag.ftcPose.yaw));
        }  else if(aprilTag.id == 4) {
            drive.setPoseEstimate(new Pose2d(aprilTag.ftcPose.x + 61, aprilTag.ftcPose.x -30, aprilTag.ftcPose.yaw));
        } else if(aprilTag.id == 5) {
            drive.setPoseEstimate(new Pose2d(aprilTag.ftcPose.x + 61, aprilTag.ftcPose.x - 36, aprilTag.ftcPose.yaw));
        } else if(aprilTag.id == 6) {
            drive.setPoseEstimate(new Pose2d(aprilTag.ftcPose.x + 61, aprilTag.ftcPose.x - 42, aprilTag.ftcPose.yaw));
        }
        adjustPosition(drive);
    }
    public void pickup() {}
    public void place() {}
}
