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

package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.cache.CacheableServo;
import org.firstinspires.ftc.teamcode.util.ConstantValues;

@Config
@TeleOp(name = "test outtake")
public class testouttake extends LinearOpMode {

    public static int state = 0;
    public static int wristState = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo slider1_servo = hardwareMap.get(Servo.class, "slider1_servo");
        Servo slider2_servo = hardwareMap.get(Servo.class, "slider2_servo");
        DcMotor slider1 = hardwareMap.get(DcMotor.class, "slider1");
        DcMotor slider2 = hardwareMap.get(DcMotor.class, "slider2");
        Servo claw1 = hardwareMap.get(Servo.class, "claw1");
        Servo claw2 = hardwareMap.get(Servo.class, "claw2");
        CacheableServo claw_wrist = new CacheableServo(hardwareMap, "claw_wrist", 0, 270);
        if (ConstantValues.INVERT_SLIDER1)
            slider1.setDirection(DcMotor.Direction.REVERSE);
        if (ConstantValues.INVERT_SLIDER2)
            slider2.setDirection(DcMotor.Direction.REVERSE);
        if (ConstantValues.INVERT_SLIDER1_SERVO)
            slider1_servo.setDirection(Servo.Direction.REVERSE);
        if (ConstantValues.INVERT_SLIDER2_SERVO)
            slider2_servo.setDirection(Servo.Direction.REVERSE);
        if (ConstantValues.INVERT_CLAW1)
            claw1.setDirection(Servo.Direction.REVERSE);
        if (ConstantValues.INVERT_CLAW2)
            claw2.setDirection(Servo.Direction.REVERSE);
        claw_wrist.setInverted(ConstantValues.INVERT_CLAW_WRIST);
        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            if (wristState == 0) {
                claw_wrist.turnToAngle(ConstantValues.CLAW_WRIST_HORIZONTAL);
            } else if (wristState == 1) {
                claw_wrist.turnToAngle(ConstantValues.CLAW_WRIST_LEFT_DIAGONAL);
            } else if (wristState == 2) {
                claw_wrist.turnToAngle(ConstantValues.CLAW_WRIST_VERTICAL);
            } else {
                claw_wrist.turnToAngle(ConstantValues.CLAW_WRIST_RIGHT_DIAGONAL);
            }
            if (state == 2) {
                slider1_servo.setPosition(ConstantValues.EXTENDED_SLIDER_SERVO_POS);
                slider2_servo.setPosition(ConstantValues.EXTENDED_SLIDER_SERVO_POS);
            } else if (state == 1) {
                slider1_servo.setPosition(ConstantValues.PICKUP_SLIDER_SERVO_POS);
                slider2_servo.setPosition(ConstantValues.PICKUP_SLIDER_SERVO_POS);
            } else {
                slider1_servo.setPosition(ConstantValues.RETRACTED_SLIDER_SERVO_POS);
                slider2_servo.setPosition(ConstantValues.RETRACTED_SLIDER_SERVO_POS);
            }
            if (gamepad2.right_trigger > 0.3) {
                //extinde
                slider1.setPower(-gamepad2.right_trigger);
                slider2.setPower(-gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0.3) {
                slider1.setPower(gamepad2.left_trigger);
                slider2.setPower(gamepad2.left_trigger);
            } else {
                slider1.setPower(0);
                slider2.setPower(0);
            }
            if (gamepad2.left_bumper) {
                claw1.setPosition(ConstantValues.CLAW_HOLD_POS);
            } else {
                claw1.setPosition(ConstantValues.CLAW_RELEASE_POS);
            }
            if (gamepad2.right_bumper) {
                claw2.setPosition(ConstantValues.CLAW_HOLD_POS);
            } else {
                claw2.setPosition(ConstantValues.CLAW_RELEASE_POS);
            }
            telemetry.addData("slider 1", slider1.getCurrentPosition());
            telemetry.update();
        }
    }
}