package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        boolean isRed = true;
        boolean isUp = false;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> {
                            Pose2d startPose;
                            Pose2d lastLeft;
                            if (!isRed) {
                                if (!isUp)
                                    startPose = new Pose2d(-36, 63, Math.toRadians(-90));
                                else
                                    startPose = new Pose2d(12, 63, Math.toRadians(-90));
                            } else {
                                if (!isUp)
                                    startPose = new Pose2d(-36, -63, Math.toRadians(90));
                                else
                                    startPose = new Pose2d(12, -63, Math.toRadians(90));
                            }
                            drive.setPoseEstimate(startPose);
                            if (!isUp) {
                                TrajectorySequence redPropRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .splineToLinearHeading(new Pose2d(isRed ? -34.00 : -37, isRed ? -34.00 : 34, Math.toRadians(isRed ? 180 : 0)), Math.toRadians(isRed ? 63 : -63))
                                        .build();
                                TrajectorySequence redPropCenter = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .splineTo(new Vector2d(-32, isRed ? -14 : 14), Math.toRadians(isRed ? 90 : -90))
                                        .build();
                                TrajectorySequence redPropCenterToStack = drive.trajectorySequenceBuilder(redPropCenter.end())
                                        .splineToLinearHeading(new Pose2d(-61, isRed ? -12 : 12, Math.toRadians(0)), Math.toRadians(180))
                                        .build();
                                TrajectorySequence redPropCenterToBoard = drive.trajectorySequenceBuilder(redPropCenterToStack.end())
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? -21 : 21))
                                        .splineTo(new Vector2d(52, isRed ? -35 : 35), Math.toRadians(0.00))
                                        .build();
                                TrajectorySequence redPropCenterFromBoardToStack = drive.trajectorySequenceBuilder(redPropCenterToBoard.end())
                                        .setReversed(true)
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? 159 : -159))
                                        .splineTo(new Vector2d(-61, isRed ? -12 : 12), Math.toRadians(180))
                                        .build();

                                TrajectorySequence redPropLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .splineToLinearHeading(new Pose2d(isRed ? -37.00 : -34, isRed ? -34.00 : 34, Math.toRadians(isRed ? 0 : 180)), Math.toRadians(isRed ? 63 : -63))
                                        .build();
                                lastLeft = redPropLeft.end();
                                TrajectorySequence redPropRightToStack = drive.trajectorySequenceBuilder(redPropRight.end())
                                        .splineToLinearHeading(new Pose2d(-61.00, isRed ? -12.00 : 12, Math.toRadians(0)), Math.toRadians(180.00))
                                        .build();
                                TrajectorySequence redPropRightToBoard = drive.trajectorySequenceBuilder(redPropRightToStack.end())
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? -21 : 21))
                                        .splineTo(new Vector2d(52.00, isRed ? -41 : 29), Math.toRadians(0.00))
                                        .build();
                                if (!isRed)
                                    redPropRightToStack = drive.trajectorySequenceBuilder(redPropRight.end())
                                            .lineTo(new Vector2d(-36, 21))
                                            .splineToLinearHeading(new Pose2d(-61, 12, Math.toRadians(0)), Math.toRadians(180))
                                            .build();
                                TrajectorySequence redPropRightFromBoardToStack = drive.trajectorySequenceBuilder(redPropRightToBoard.end())
                                        .setReversed(true)
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? 159 : -159))
                                        .splineTo(new Vector2d(-61, isRed ? -12 : 12), Math.toRadians(180))
                                        .build();
                                TrajectorySequence redPropLeftToStack = drive.trajectorySequenceBuilder(lastLeft)
                                        .lineTo(new Vector2d(-36, isRed ? -21 : 21))
                                        .splineToLinearHeading(new Pose2d(-61, isRed ? -12 : 12, Math.toRadians(0)), Math.toRadians(180))
                                        .build();
                                if (!isRed)
                                    redPropLeftToStack = drive.trajectorySequenceBuilder(lastLeft)
                                            .splineToLinearHeading(new Pose2d(-61.00, 12.00, Math.toRadians(0)), Math.toRadians(180.00))
                                            .build();
                                lastLeft = redPropLeftToStack.end();
                                TrajectorySequence redPropLeftToBoard = drive.trajectorySequenceBuilder(lastLeft)
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? -21 : 21))
                                        .splineTo(new Vector2d(52, isRed ? -29 : 41), Math.toRadians(0.00))
                                        .build();
                                lastLeft = redPropLeftToBoard.end();
                                TrajectorySequence redPropLeftFromBoardToStack = drive.trajectorySequenceBuilder(lastLeft)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? 159 : -159))
                                        .splineTo(new Vector2d(-61, isRed ? -12 : 12), Math.toRadians(180))
                                        .build();
                                return redPropRightToStack;
                            } else {
                                TrajectorySequence redPropRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .splineToLinearHeading(new Pose2d(isRed ? 14.00 : 11, isRed ? -34.00 : 34, Math.toRadians(isRed ? 180 : 0)), Math.toRadians(isRed ? 126 : -63))
                                        .build();
                                TrajectorySequence redPropCenter = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .splineTo(new Vector2d(16, isRed ? -14 : 14), Math.toRadians(isRed ? 90 : -90))
                                        .build();
                                TrajectorySequence redPropCenterToStack = drive.trajectorySequenceBuilder(redPropCenter.end())
                                        .splineToLinearHeading(new Pose2d(-61, isRed ? -12 : 12, Math.toRadians(0)), Math.toRadians(180))
                                        .build();
                                TrajectorySequence redPropCenterToBoard = drive.trajectorySequenceBuilder(redPropCenter.end())
                                        .splineTo(new Vector2d(30, isRed ? -16 : 16), Math.toRadians(isRed ? -45.00 : 45))
                                        .splineTo(new Vector2d(52, isRed ? -35 : 35), Math.toRadians(0.00))
                                        .build();
                                TrajectorySequence redPropCenterFromBoardToStack = drive.trajectorySequenceBuilder(redPropCenterToBoard.end())
                                        .setReversed(true)
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? 159 : -159))
                                        .splineTo(new Vector2d(-61, isRed ? -12 : 12), Math.toRadians(180))
                                        .build();

                                TrajectorySequence redPropLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .splineToLinearHeading(new Pose2d(isRed ? 11 : 14, isRed ? -34.00 : 34, Math.toRadians(isRed ? 0 : 180)), Math.toRadians(isRed ? 126 : 63))
                                        .build();
                                TrajectorySequence redPropRightToStack = drive.trajectorySequenceBuilder(redPropRight.end())
                                        .splineToLinearHeading(new Pose2d(-61.00, isRed ? -12.00 : 12, Math.toRadians(0)), Math.toRadians(180.00))
                                        .build();
                                TrajectorySequence redPropRightToBoard = drive.trajectorySequenceBuilder(redPropRight.end())
                                        .splineTo(new Vector2d(12, isRed ? -41 : 41), Math.toRadians(isRed ? -21 : 21))
                                        .splineTo(new Vector2d(52.00, isRed ? -41 : 29), Math.toRadians(0.00))
                                        .build();
                                TrajectorySequence redPropRightFromBoardToStack = drive.trajectorySequenceBuilder(redPropRightToBoard.end())
                                        .setReversed(true)
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? 159 : -159))
                                        .splineTo(new Vector2d(-61, isRed ? -12 : 12), Math.toRadians(180))
                                        .build();

                                TrajectorySequence redPropLeftToStack = drive.trajectorySequenceBuilder(redPropLeft.end())
                                        .lineTo(new Vector2d(-36, isRed ? -21 : 21))
                                        .splineToLinearHeading(new Pose2d(-61, isRed ? -12 : 12, Math.toRadians(0)), Math.toRadians(180))
                                        .build();
                                if (!isRed)
                                    redPropRightToBoard = drive.trajectorySequenceBuilder(redPropRight.end())
                                            .splineTo(new Vector2d(52, isRed ? -29 : 41), Math.toRadians(0.00))
                                            .build();
                                TrajectorySequence redPropLeftToBoard = drive.trajectorySequenceBuilder(redPropLeft.end())
                                        .splineTo(new Vector2d(52, isRed ? -29 : 41), Math.toRadians(0.00))
                                        .build();
                                if (!isRed)
                                    redPropLeftToBoard = drive.trajectorySequenceBuilder(redPropLeft.end())
                                            .splineTo(new Vector2d(12, isRed ? -41 : 41), Math.toRadians(isRed ? -21 : 21))
                                            .splineTo(new Vector2d(52.00, isRed ? -41 : 29), Math.toRadians(0.00))
                                            .build();
                                TrajectorySequence redPropLeftFromBoardToStack = drive.trajectorySequenceBuilder(redPropLeftToBoard.end())
                                        .setReversed(true)
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? 159 : -159))
                                        .splineTo(new Vector2d(-61, isRed ? -12 : 12), Math.toRadians(180))
                                        .build();
                                TrajectorySequence redPropLeftFromStackToBoard = drive.trajectorySequenceBuilder(redPropLeftToStack.end())
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? -21 : 21))
                                        .splineTo(new Vector2d(52, isRed ? -29 : 41), Math.toRadians(0.00))
                                        .build();
                                TrajectorySequence redPropCenterFromStackToBoard = drive.trajectorySequenceBuilder(redPropCenterToStack.end())
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? -21 : 21))
                                        .splineTo(new Vector2d(52, isRed ? -35 : 35), Math.toRadians(0.00))
                                        .build();
                                TrajectorySequence redPropRightFromStackToBoard = drive.trajectorySequenceBuilder(redPropRightToStack.end())
                                        .splineTo(new Vector2d(25, isRed ? -18 : 18), Math.toRadians(isRed ? -21 : 21))
                                        .splineTo(new Vector2d(52.00, isRed ? -41 : 29), Math.toRadians(0.00))
                                        .build();
                                return redPropCenterFromStackToBoard;
                            }
                        }
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
