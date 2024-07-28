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

package org.firstinspires.ftc.teamcode.handlers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cache.CacheManager;
import org.firstinspires.ftc.teamcode.commands.ActivateIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.AlignWristHorizontallyCommand;
import org.firstinspires.ftc.teamcode.commands.AlignWristToLeftCommand;
import org.firstinspires.ftc.teamcode.commands.AlignWristToRightCommand;
import org.firstinspires.ftc.teamcode.commands.AlignWristVerticallyCommand;
import org.firstinspires.ftc.teamcode.commands.DeactivateIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendOuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendSlidersCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchDroneCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakePickupCommand;
import org.firstinspires.ftc.teamcode.commands.ReleaseClaw1Command;
import org.firstinspires.ftc.teamcode.commands.ReleaseClaw2Command;
import org.firstinspires.ftc.teamcode.commands.ResetDroneCommand;
import org.firstinspires.ftc.teamcode.commands.RetractIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.RetractSlidersCommand;
import org.firstinspires.ftc.teamcode.commands.StandBySlidersCommand;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SliderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.ConstantValues;

public class HandleTeleOp {
    public static long delta;
    private static GamepadEx driver;
    private static GamepadEx tool;
    private static FieldCentricDriveSubsystem fieldCentricDriveSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static SliderSubsystem sliderSubsystem;
    private static WristSubsystem wristSubsystem;
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
        fieldCentricDriveSubsystem = new FieldCentricDriveSubsystem(
                hardwareMap, driver, isRed, opMode
        );
        intakeSubsystem = new IntakeSubsystem(hardwareMap, tool, driver);
        sliderSubsystem = new SliderSubsystem(hardwareMap, tool);
        wristSubsystem = new WristSubsystem(hardwareMap, tool);
        droneSubsystem = new DroneSubsystem(hardwareMap, driver);
        driver.getGamepadButton(ConstantValues.DRONE).whenHeld(new LaunchDroneCommand(droneSubsystem)).whenReleased(new ResetDroneCommand(droneSubsystem));
        tool.getGamepadButton(ConstantValues.TOGGLE_OUTTAKE).toggleWhenPressed(new ExtendOuttakeCommand(sliderSubsystem, wristSubsystem), new RetractOuttakeCommand(sliderSubsystem, wristSubsystem));
        tool.getGamepadButton(ConstantValues.INTAKE).and(new Trigger(() -> !intakeSubsystem.pixel1 || !intakeSubsystem.pixel2)).whenActive(new ActivateIntakeCommand(intakeSubsystem, false, false)).whenInactive(new DeactivateIntakeCommand(intakeSubsystem));
        tool.getGamepadButton(ConstantValues.REVERSE_INTAKE).whenHeld(new ActivateIntakeCommand(intakeSubsystem, true, false)).whenReleased(new DeactivateIntakeCommand(intakeSubsystem));
        tool.getGamepadButton(ConstantValues.EXTEND_INTAKE).whenHeld(new ExtendIntakeCommand(intakeSubsystem)).whenReleased(new RetractIntakeCommand(intakeSubsystem));
        tool.getGamepadButton(ConstantValues.CLAW_1).whenPressed(new ReleaseClaw1Command(wristSubsystem));
        tool.getGamepadButton(ConstantValues.CLAW_2).whenPressed(new ReleaseClaw2Command(wristSubsystem));
        new Trigger(() -> tool.getRightX() >= 0.7).whenActive(new AlignWristToRightCommand(wristSubsystem));
        new Trigger(() -> tool.getRightX() <= -0.7).whenActive(new AlignWristToLeftCommand(wristSubsystem));
        new Trigger(() -> tool.getRightY() >= 0.7).whenActive(new AlignWristHorizontallyCommand(wristSubsystem));
        new Trigger(() -> tool.getRightY() <= -0.7).whenActive(new AlignWristVerticallyCommand(wristSubsystem));
        new Trigger(() -> tool.getTrigger(ConstantValues.EXTEND_SLIDERS) >= 0.1 && !sliderSubsystem.isExtended()).whenActive(new ExtendSlidersCommand(sliderSubsystem, tool.getTrigger(ConstantValues.EXTEND_SLIDERS))).whenInactive(new StandBySlidersCommand(sliderSubsystem));
        new Trigger(() -> tool.getTrigger(ConstantValues.RETRACT_SLIDERS) >= 0.1 && !sliderSubsystem.isRetracted()).whenActive(new RetractSlidersCommand(sliderSubsystem, tool.getTrigger(ConstantValues.RETRACT_SLIDERS))).whenInactive(new StandBySlidersCommand(sliderSubsystem));
        opMode.register(fieldCentricDriveSubsystem, intakeSubsystem, sliderSubsystem, droneSubsystem);
        fieldCentricDriveSubsystem.setDefaultCommand(new DriveCommand(fieldCentricDriveSubsystem));
        sliderSubsystem.setTelemetry(telemetry);
        opMode.schedule(new RunCommand(telemetry::update));
        new Trigger(() -> !intakeSubsystem.oldPixel1 && intakeSubsystem.pixel1 && !intakeSubsystem.oldPixel2 && intakeSubsystem.pixel2 && sliderSubsystem.isRetracted() && sliderSubsystem.isArmRetracted()).whenActive(new OuttakePickupCommand(sliderSubsystem, wristSubsystem));
    }

    public static void run() {
        cacheManager.clear();
        currentTime = System.currentTimeMillis();
        delta = currentTime - lastTime;
        lastTime = currentTime;
        telemetry.addData("time diff", delta);
    }
}