package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AdjustPositionSubsystem extends SubsystemBase {
    private SampleMecanumDrive drive;
    private AprilTagProcessor processor;
    public AdjustPositionSubsystem(SampleMecanumDrive drive, AprilTagProcessor processor) {
        this.drive = drive;
        this.processor = processor;
    }
    public void adjustPosition() {
        ArrayList<AprilTagDetection> aprilTags = processor.getDetections();
        if(aprilTags.size() == 0)
            return;
        //calculate for apriltags (test april tag angle)
        AprilTagDetection aprilTag = aprilTags.get(0);
        if(aprilTag.id == 7 || aprilTag.id == 8 || aprilTag.id == 9 || aprilTag.id == 10) {
            drive.setPoseEstimate(new Pose2d(aprilTag.metadata.fieldPosition.get(0) + aprilTag.ftcPose.y, -aprilTag.ftcPose.x + aprilTag.metadata.fieldPosition.get(1), aprilTag.ftcPose.yaw));
        } else
            drive.setPoseEstimate(new Pose2d(aprilTag.metadata.fieldPosition.get(0) - aprilTag.ftcPose.y, aprilTag.ftcPose.x + aprilTag.metadata.fieldPosition.get(1), aprilTag.ftcPose.yaw));
    }
}
