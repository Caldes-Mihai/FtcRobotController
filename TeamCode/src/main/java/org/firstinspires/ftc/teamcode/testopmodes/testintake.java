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

import org.firstinspires.ftc.teamcode.util.ConstantValues;

@TeleOp(name = "test intake")
@Config
public class testintake extends LinearOpMode {

    public static double retract = 0;
    public static double extend = 0.3;
    public static double stage = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo intake_servo = hardwareMap.get(Servo.class, "intake_servo");

//        Servo claw1 = hardwareMap.get(Servo.class, "claw1");
//        Servo claw2 = hardwareMap.get(Servo.class, "claw2");
//        NormalizedColorSensor pixel1 = hardwareMap.get(NormalizedColorSensor.class, "pixel1");
//        NormalizedColorSensor pixel2 = hardwareMap.get(NormalizedColorSensor.class, "pixel2");
//        boolean isPixel1 = ((DistanceSensor) pixel1).getDistance(DistanceUnit.CM) < ConstantValues.PIXEL_DISTANCE_THRESHOLD;
//        boolean isPixel2 = ((DistanceSensor) pixel2).getDistance(DistanceUnit.CM) < ConstantValues.PIXEL_DISTANCE_THRESHOLD;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up)
                intake_servo.setPosition(ConstantValues.INTAKE_SERVO_EXTEND_POS);
            else
                intake_servo.setPosition(ConstantValues.INTAKE_SERVO_RETRACT_POS);
            if (gamepad2.x) {
                intake.setPower(-1);
            } else if (gamepad2.b) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
//            if (isPixel1 && isPixel2) {
//                claw1.setPosition(ConstantValues.CLAW1_HOLD_POS);
//                claw2.setPosition(ConstantValues.CLAW2_HOLD_POS);
//            } else {
//                claw1.setPosition(ConstantValues.CLAW1_RELEASE_POS);
//                claw2.setPosition(ConstantValues.CLAW2_RELEASE_POS);
//            }
//            telemetry.addData("pixel1", isPixel1);
//            telemetry.addData("pixel2", isPixel2);
//            telemetry.update();
        }
    }
}