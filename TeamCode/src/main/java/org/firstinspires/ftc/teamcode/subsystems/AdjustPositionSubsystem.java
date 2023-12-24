package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AdjustPositionSubsystem extends SubsystemBase {
    private final SampleMecanumDrive drive;
    private final AprilTagProcessor processor;
    private final boolean isRed;
    //camera offset from center (in inches)
    private final double offsetY = 12;
    private final double offsetX = 6;
    private double heading;
    private double x;
    private double y;
    private double size;
    private double sumX;
    private double sumY;
    private double sumHeading;

    public AdjustPositionSubsystem(SampleMecanumDrive drive, AprilTagProcessor processor, boolean isRed) {
        this.drive = drive;
        this.processor = processor;
        this.isRed = isRed;
    }

    public void adjustPosition() {
        ArrayList<AprilTagDetection> aprilTags = processor.getDetections();
        size = aprilTags.size();
        if (size == 0)
            return;
        //TODO: not sure if heading readjusting works properly (might need to swap + with -)
        sumX = 0;
        sumY = 0;
        sumHeading = 0;
        aprilTags.forEach(aprilTag -> {
            if (aprilTag.id == 7 || aprilTag.id == 8 || aprilTag.id == 9 || aprilTag.id == 10) {
                if (isRed) {
                    heading = -90 + aprilTag.ftcPose.yaw;
                } else {
                    heading = 90 + aprilTag.ftcPose.yaw;
                }
            } else {
                if (isRed) {
                    heading = 90 + aprilTag.ftcPose.yaw;
                } else {
                    heading = -90 + aprilTag.ftcPose.yaw;
                }
            }
            x = aprilTag.metadata.fieldPosition.get(0);
            y = aprilTag.metadata.fieldPosition.get(1);
            x = x - Math.signum(x) * aprilTag.ftcPose.y;
            y = y - Math.signum(y) * aprilTag.ftcPose.x;
            x = x - Math.sin(Math.toRadians(heading)) * offsetY - Math.cos(Math.toRadians(heading)) * offsetX;
            y = y - Math.cos(Math.toRadians(heading)) * offsetY - Math.sin(Math.toRadians(heading)) * offsetX;
            sumX += x;
            sumY += y;
            sumHeading += heading;
        });
        drive.setPoseEstimate(new Pose2d(sumX / size, sumY / size, Math.toRadians(sumHeading / size)));
    }
}
