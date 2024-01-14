/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.cache.CacheManager;
import org.firstinspires.ftc.teamcode.cache.CacheableCRServo;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.HandleIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.HandleOuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TeleOpDriveSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class HandleTeleOp {
    public static long delta;
    private static IMU imu;
    private static CacheableMotor leftFrontDrive;
    private static CacheableMotor leftBackDrive;
    private static CacheableMotor rightFrontDrive;
    private static CacheableMotor rightBackDrive;
    private static CacheableMotor intake;
    private static CacheableMotor slider1;
    private static CacheableMotor slider2;
    private static CacheableCRServo holder;
    private static AprilTagProcessor processor;
    private static VisionPortal visionPortal;
    private static GamepadEx driver;
    private static GamepadEx tool;
    private static IMU.Parameters parameters;
    private static TeleOpDriveSubsystem teleOpDriveSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static OuttakeSubsystem outtakeSubsystem;
    private static CommandOpMode opMode;
    private static HardwareMap hardwareMap;
    private static Telemetry telemetry;
    private static CacheManager cacheManager;
    private static long currentTime;
    private static long lastTime;

    public static void init(boolean isRed, CommandOpMode op) {
        opMode = op;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        cacheManager = new CacheManager(hardwareMap);
        driver = new GamepadEx(opMode.gamepad1);
        tool = new GamepadEx(opMode.gamepad2);
        imu = hardwareMap.get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        processor = new AprilTagProcessor.Builder().build();
        processor.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .build();
        setManualExposure(6, 250);
        leftFrontDrive = new CacheableMotor(hardwareMap, "front_left_motor");
        leftBackDrive = new CacheableMotor(hardwareMap, "back_left_motor");
        rightFrontDrive = new CacheableMotor(hardwareMap, "front_right_motor");
        rightBackDrive = new CacheableMotor(hardwareMap, "back_right_motor");
        intake = new CacheableMotor(hardwareMap, "intake");
        slider1 = new CacheableMotor(hardwareMap, "slider1");
        slider2 = new CacheableMotor(hardwareMap, "slider2");
        holder = new CacheableCRServo(hardwareMap, "holder");
        teleOpDriveSubsystem = new TeleOpDriveSubsystem(
                leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, imu, processor,
                driver, isRed, opMode
        );
        intakeSubsystem = new IntakeSubsystem(intake, tool);
        outtakeSubsystem = new OuttakeSubsystem(slider1, slider2, holder, tool);
        opMode.register(teleOpDriveSubsystem, intakeSubsystem, outtakeSubsystem);
        teleOpDriveSubsystem.setDefaultCommand(new DriveCommand(teleOpDriveSubsystem));
        intakeSubsystem.setDefaultCommand(new HandleIntakeCommand(intakeSubsystem));
        outtakeSubsystem.setDefaultCommand(new HandleOuttakeCommand(outtakeSubsystem));
        opMode.schedule(new RunCommand(telemetry::update));
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

    public static void run() {
        cacheManager.clear();
        currentTime = System.currentTimeMillis();
        delta = currentTime - lastTime;
        lastTime = currentTime;
        telemetry.addData("time", 1000 / delta + "hz");
        telemetry.update();
    }
}