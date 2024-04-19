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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.drive.ConstantValues;

@TeleOp(name = "test color")
@Config
public class testcolor extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor pixel1 = hardwareMap.get(NormalizedColorSensor.class, "pixel1");
        NormalizedRGBA rgba = pixel1.getNormalizedColors();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("red", rgba.red * 255);
            telemetry.addData("green", rgba.green * 255);
            telemetry.addData("blue", rgba.blue * 255);
            telemetry.addData("pixel", getPixel(pixel1).toString());
            telemetry.update();
        }
    }

    public Pixels getPixel(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        if (withinRange(rgba.red * 255, ConstantValues.WHITE_PIXEL[0], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.green * 255, ConstantValues.WHITE_PIXEL[1], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.blue * 255, ConstantValues.WHITE_PIXEL[2], ConstantValues.PIXEL_COLOR_THRESHOLD))
            return Pixels.WHITE;
        else if (withinRange(rgba.red * 255, ConstantValues.YELLOW_PIXEL[0], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.green * 255, ConstantValues.YELLOW_PIXEL[1], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.blue * 255, ConstantValues.YELLOW_PIXEL[2], ConstantValues.PIXEL_COLOR_THRESHOLD))
            return Pixels.YELLOW;
        else if (withinRange(rgba.red * 255, ConstantValues.GREEN_PIXEL[0], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.green * 255, ConstantValues.GREEN_PIXEL[1], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.blue * 255, ConstantValues.GREEN_PIXEL[2], ConstantValues.PIXEL_COLOR_THRESHOLD))
            return Pixels.GREEN;
        else if (withinRange(rgba.red * 255, ConstantValues.PURPLE_PIXEL[0], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.green * 255, ConstantValues.PURPLE_PIXEL[1], ConstantValues.PIXEL_COLOR_THRESHOLD) && withinRange(rgba.blue * 255, ConstantValues.PURPLE_PIXEL[2], ConstantValues.PIXEL_COLOR_THRESHOLD))
            return Pixels.PURPLE;
        return Pixels.NONE;
    }

    private boolean withinRange(double input1, double input2, double deviation) {
        return Math.abs(input1 - input2) <= deviation;
    }

    private enum Pixels {
        WHITE,
        PURPLE,
        GREEN,
        YELLOW,
        NONE
    }
}