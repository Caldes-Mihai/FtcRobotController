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
    }
}
