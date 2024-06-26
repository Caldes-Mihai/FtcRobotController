package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.handlers.HandleAuto;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.ConstantValues;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * A subsystem that uses the {@link SampleMecanumDrive} class.
 * This periodically calls {@link SampleMecanumDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class AutoDriveSubsystem extends SubsystemBase {

    private final SampleMecanumDrive drive;
    private final boolean fieldCentric = false;
    private final AprilTagProcessor processor;
    //camera offset from center (in inches)
    private double heading;
    private double x;
    private double y;
    private double size;
    private double sumX;
    private double sumY;
    private TrajectorySequence leftTrajectory;
    private TrajectorySequence centerTrajectory;
    private TrajectorySequence rightTrajectory;

    public AutoDriveSubsystem(HardwareMap hardwareMap, AprilTagProcessor processor, HandleAuto.Positions currentSpawnPosition, boolean isRed) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        if (!isRed) {
            if (currentSpawnPosition.equals(HandleAuto.Positions.DOWN))
                drive.setPoseEstimate(new Pose2d(-36, 63, Math.toRadians(-90)));
            else
                drive.setPoseEstimate(new Pose2d(12, 63, Math.toRadians(-90)));
        } else {
            if (currentSpawnPosition.equals(HandleAuto.Positions.DOWN))
                drive.setPoseEstimate(new Pose2d(-36, -63, Math.toRadians(90)));
            else
                drive.setPoseEstimate(new Pose2d(12, -63, Math.toRadians(90)));
        }
        this.processor = processor;
    }

    public TrajectorySequence getLeftTrajectory() {
        return leftTrajectory;
    }

    public void setLeftTrajectory(TrajectorySequence trajectory) {
        leftTrajectory = trajectory;
    }

    public TrajectorySequence getCenterTrajectory() {
        return centerTrajectory;
    }

    public void setCenterTrajectory(TrajectorySequence trajectory) {
        centerTrajectory = trajectory;
    }

    public TrajectorySequence getRightTrajectory() {
        return rightTrajectory;
    }

    public void setRightTrajectory(TrajectorySequence trajectory) {
        rightTrajectory = trajectory;
    }

    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
    }

    public void update() {
        drive.update();
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX) {
        Pose2d poseEstimate = getPoseEstimate();

        Vector2d input = new Vector2d(-leftY, -leftX).rotated(
                fieldCentric ? -poseEstimate.getHeading() : 0
        );

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -rightX
                )
        );
    }

    public void adjustPosition() {
        ArrayList<AprilTagDetection> aprilTags = processor.getDetections();
        size = aprilTags.size();
        if (size == 0)
            return;
        sumX = 0;
        sumY = 0;
        heading = drive.getRawExternalHeading();
        aprilTags.forEach(aprilTag -> {
            x = aprilTag.metadata.fieldPosition.get(0);
            y = aprilTag.metadata.fieldPosition.get(1);
            x = x - Math.signum(x) * aprilTag.ftcPose.y;
            y = y - Math.signum(y) * aprilTag.ftcPose.x;
            x = x - Math.cos(Math.toRadians(heading)) * ConstantValues.CAMERA_OFFSET_Y - Math.sin(Math.toRadians(heading)) * ConstantValues.CAMERA_OFFSET_X;
            y = y - Math.sin(Math.toRadians(heading)) * ConstantValues.CAMERA_OFFSET_Y - Math.cos(Math.toRadians(heading)) * ConstantValues.CAMERA_OFFSET_X;
            sumX += x;
            sumY += y;
        });
        drive.setPoseEstimate(new Pose2d(sumX / size, sumY / size, heading));
    }

    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public void followTrajectory(TrajectorySequence trajectory) {
        drive.followTrajectorySequenceAsync(trajectory);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d pose) {
        return drive.trajectorySequenceBuilder(pose);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

}