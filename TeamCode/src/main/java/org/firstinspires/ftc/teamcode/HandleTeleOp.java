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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cache.CacheManager;
import org.firstinspires.ftc.teamcode.cache.CacheableCRServo;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;
import org.firstinspires.ftc.teamcode.cache.CacheableServo;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.HandleDroneCommand;
import org.firstinspires.ftc.teamcode.commands.HandleIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.HandleOuttakeCommand;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TeleOpDriveSubsystem;

public class HandleTeleOp {
    public static long delta;
    private static IMU imu;
    private static CacheableMotor leftFrontDrive;
    private static CacheableMotor leftBackDrive;
    private static CacheableMotor rightFrontDrive;
    private static CacheableMotor rightBackDrive;
    private static CacheableMotor intake;
    private static CacheableServo intake_servo;
    private static CacheableMotor slider1;
    private static CacheableMotor slider2;
    private static CacheableServo slider1_servo;
    private static CacheableServo slider2_servo;
    private static CacheableServo drone;
    private static CacheableCRServo holder;
    private static GamepadEx driver;
    private static GamepadEx tool;
    private static IMU.Parameters parameters;
    private static TeleOpDriveSubsystem teleOpDriveSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static OuttakeSubsystem outtakeSubsystem;
    private static DroneSubsystem droneSubsystem;
    private static CommandOpMode opMode;
    private static HardwareMap hardwareMap;
    private static Telemetry telemetry;
    private static CacheManager cacheManager;
    private static long currentTime;
    private static long lastTime;

    public static void init(boolean isRed, CommandOpMode op) {
        opMode = op;
        hardwareMap = opMode.hardwareMap;
        telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        opMode.telemetry = telemetry;
        cacheManager = new CacheManager(hardwareMap);
        driver = new GamepadEx(opMode.gamepad1);
        tool = new GamepadEx(opMode.gamepad2);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR,
                DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);
        leftFrontDrive = new CacheableMotor(hardwareMap, "front_left_motor");
        leftBackDrive = new CacheableMotor(hardwareMap, "back_left_motor");
        rightFrontDrive = new CacheableMotor(hardwareMap, "front_right_motor");
        rightBackDrive = new CacheableMotor(hardwareMap, "back_right_motor");
        intake = new CacheableMotor(hardwareMap, "intake");
        intake_servo = new CacheableServo(hardwareMap, "intake_servo", 0, 180);
        slider1 = new CacheableMotor(hardwareMap, "slider1");
        slider2 = new CacheableMotor(hardwareMap, "slider2");
        slider1_servo = new CacheableServo(hardwareMap, "slider1_servo", 0, 270);
        slider2_servo = new CacheableServo(hardwareMap, "slider2_servo", 0, 270);
        holder = new CacheableCRServo(hardwareMap, "holder");
        drone = new CacheableServo(hardwareMap, "drone", 0, 270);
        teleOpDriveSubsystem = new TeleOpDriveSubsystem(
                leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, imu,
                driver, isRed, opMode
        );
        intakeSubsystem = new IntakeSubsystem(intake, intake_servo, null, tool);
        outtakeSubsystem = new OuttakeSubsystem(slider1, slider2, slider1_servo, slider2_servo, holder, tool);
        droneSubsystem = new DroneSubsystem(drone, driver);
        opMode.register(teleOpDriveSubsystem, intakeSubsystem, outtakeSubsystem, droneSubsystem);
        teleOpDriveSubsystem.setDefaultCommand(new DriveCommand(teleOpDriveSubsystem));
        intakeSubsystem.setDefaultCommand(new HandleIntakeCommand(intakeSubsystem));
        outtakeSubsystem.setDefaultCommand(new HandleOuttakeCommand(outtakeSubsystem));
        droneSubsystem.setDefaultCommand(new HandleDroneCommand(droneSubsystem));
        opMode.schedule(new RunCommand(telemetry::update));
    }

    public static void run() {
        cacheManager.clear();
        currentTime = System.currentTimeMillis();
        delta = currentTime - lastTime;
        lastTime = currentTime;
        telemetry.addData("time diff", delta);
    }
}