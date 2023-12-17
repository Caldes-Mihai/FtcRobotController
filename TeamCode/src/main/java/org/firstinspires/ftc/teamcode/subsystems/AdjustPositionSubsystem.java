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
    private boolean isRed;
    private double heading;
    private double x;
    private double y;
    private double size;
    private double sumX;
    private double sumY;
    private double sumHeading;
    //camera offset from center (in inches)
    private double offset = 12;
    public AdjustPositionSubsystem(SampleMecanumDrive drive, AprilTagProcessor processor, boolean isRed) {
        this.drive = drive;
        this.processor = processor;
        this.isRed = isRed;
    }
    public void adjustPosition() {
        ArrayList<AprilTagDetection> aprilTags = processor.getDetections();
        size = aprilTags.size();
        if(size == 0)
            return;
        //TO DO: not sure if heading readjusting works properly (might need to swap + with -)
        sumX = 0;
        sumY = 0;
        sumHeading = 0;
        aprilTags.forEach(aprilTag -> {
            if(aprilTag.id == 7 || aprilTag.id == 8 || aprilTag.id == 9 || aprilTag.id == 10) {
                if(isRed) {
                    heading = -90 + aprilTag.ftcPose.yaw;
                } else {
                    heading = 90 + aprilTag.ftcPose.yaw;
                }
                x =  aprilTag.metadata.fieldPosition.get(0) + aprilTag.ftcPose.y;
                y = -aprilTag.ftcPose.x + aprilTag.metadata.fieldPosition.get(1);
                x = x - Math.sin(Math.toRadians(heading)) * offset;
                y = y - Math.cos(Math.toRadians(heading)) * offset;
            } else {
                if(isRed) {
                    heading = 90 + aprilTag.ftcPose.yaw;
                } else {
                    heading = -90 + aprilTag.ftcPose.yaw;
                }
                x = aprilTag.metadata.fieldPosition.get(0) - aprilTag.ftcPose.y;
                y = aprilTag.ftcPose.x + aprilTag.metadata.fieldPosition.get(1);
                x = x - Math.sin(Math.toRadians(heading)) * offset;
                y = y - Math.cos(Math.toRadians(heading)) * offset;
            }
            sumX += x;
            sumY += y;
            sumHeading += heading;
        });
        drive.setPoseEstimate(new Pose2d(sumX / size, sumY / size, Math.toRadians(sumHeading / size)));
    }
}
