/*
 * Copyright (c) 2023 Sebastian Erives
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

package org.firstinspires.ftc.teamcode.processor;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PredictMozaicProcessor implements VisionProcessor {

    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Scalars in OpenCV are generally used to
     * represent color. So our values in the
     * lower and upper Scalars here represent
     * the Y, Cr and Cb values respectively.
     *
     * YCbCr, like most color spaces, range
     * from 0-255, so we default to those
     * min and max values here for now, meaning
     * that all pixels will be shown.
     */
    public Scalar lowerBlue = new Scalar(90, 102, 166);
    public Scalar upperBlue = new Scalar(100, 255, 255);
    public Scalar lowerPurple = new Scalar(0, 0, 135);
    public Scalar upperPurple = new Scalar(162, 39, 255);
    public Scalar lowerGreen = new Scalar(0, 0, 135);
    public Scalar upperGreen = new Scalar(162, 39, 255);
    public Scalar lowerYellow = new Scalar(0, 0, 135);
    public Scalar upperYellow = new Scalar(162, 39, 255);
    public Point areaStart = new Point(180, 270);
    public Point areaEnd = new Point(540, 480);
    public int maxHWDiff = 30;
    public int treshold = 400;
    public int limit = 500;
    public boolean isRed;
    public enum Positions {
        LEFT,
        CENTER,
        RIGHT
    }

    Positions position = Positions.LEFT;
    int aprilTag;
    Rect foundBluePixel;
    Rect foundPurplePixel;
    Rect foundGreenPixel;
    Rect foundYellowPixel;
    /**
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */

    /*
     * A good practice when typing EOCV pipelines is
     * declaring the Mats you will use here at the top
     * of your pipeline, to reuse the same buffers every
     * time. This removes the need to call mat.release()
     * with every Mat you create on the processFrame method,
     * and therefore, reducing the possibility of getting a
     * memory leak and causing the app to crash due to an
     * "Out of Memory" error.
     */
    private Mat hsvMat = new Mat();
    private Mat blueMat = new Mat();
    private Mat purpleMat = new Mat();
    private Mat greenMat = new Mat();
    private Mat yellowMat = new Mat();
    private Telemetry t;
    private Paint rectPaint = new Paint();
    private Scalar textColor = new Scalar(0, 0, 0);
    /**
     * Enum to choose which color space to choose
     * with the live variable tuner isntead of
     * hardcoding it.
     */

    public PredictMozaicProcessor(Telemetry t) {
        this.t = t;
    }
    public void setRed(boolean isRed) {
        this.isRed = isRed;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lowerBlue, upperBlue, blueMat);
        Core.inRange(hsvMat, lowerPurple, upperPurple, purpleMat);
        Core.inRange(hsvMat, lowerGreen, upperGreen, greenMat);
        Core.inRange(hsvMat, lowerYellow, upperYellow, yellowMat);
        List<MatOfPoint> blueContours = new ArrayList<>();
        List<MatOfPoint> purpleContours = new ArrayList<>();
        List<MatOfPoint> greenContours = new ArrayList<>();
        List<MatOfPoint> yellowContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        try {
            Imgproc.findContours(blueMat, blueContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(purpleMat, purpleContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(greenMat, greenContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(yellowMat, yellowContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            for(Rect foundPixel : new Rect[]{foundBluePixel, foundPurplePixel, foundGreenPixel, foundYellowPixel}) {
                if(foundPixel == foundBluePixel)
                    textColor = new Scalar(0, 0, 255);
                else if(foundPixel == foundPurplePixel)
                    textColor = new Scalar(255, 0, 255);
                else if(foundPixel == foundGreenPixel)
                    textColor = new Scalar(0, 255, 0);
                else if(foundPixel == foundYellowPixel)
                    textColor = new Scalar(255, 255, 0);
                if (foundPixel != null) {
                    int baseline[] = {0};
                    Size textSize = Imgproc.getTextSize(position.toString() + "-0" + aprilTag, Imgproc.FONT_HERSHEY_PLAIN, 1, 2, baseline);
                    Point pos = new Point(foundPixel.x + (foundPixel.width - textSize.width) / 2, foundPixel.y + foundPixel.height + textSize.height + 4);
                    Imgproc.putText(frame, position.toString() + "-0" + aprilTag, pos, Imgproc.FONT_HERSHEY_PLAIN, 1, textColor, 2);
                }
            }
        } catch (Exception e) {}
        blueMat.release();
        purpleMat.release();
        greenMat.release();
        yellowMat.release();
        return new ArrayList<>(Arrays.asList(blueContours, purpleContours, greenContours, yellowContours));
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        rectPaint.setColor(isRed ? Color.RED : Color.BLUE);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
        ArrayList<ArrayList<MatOfPoint>> allContours = (ArrayList) userContext;
        ArrayList<MatOfPoint> blueContours = allContours.get(0);
        ArrayList<MatOfPoint> purpleContours = allContours.get(1);
        ArrayList<MatOfPoint> greenContours = allContours.get(2);
        ArrayList<MatOfPoint> yellowContours = allContours.get(3);
        foundBluePixel = foundPurplePixel = foundGreenPixel = foundYellowPixel = null;
        findBlue(blueContours, scaleBmpPxToCanvasPx);
        findPurple(purpleContours, scaleBmpPxToCanvasPx);
        findGreen(greenContours, scaleBmpPxToCanvasPx);
        findYellow(yellowContours, scaleBmpPxToCanvasPx);
        Rect area = new Rect(areaStart, areaEnd);
        for(Rect foundPixel : new Rect[]{foundBluePixel, foundPurplePixel, foundGreenPixel, foundYellowPixel}) {
            if(foundPixel == foundBluePixel)
                rectPaint.setColor(Color.BLUE);
            else if(foundPixel == foundPurplePixel)
                rectPaint.setColor(Color.MAGENTA);
            else if(foundPixel == foundGreenPixel)
                rectPaint.setColor(Color.GREEN);
            else if(foundPixel == foundYellowPixel)
                rectPaint.setColor(Color.YELLOW);
            if (foundPixel != null) {
                if (foundPixel.x - area.x + foundPixel.width / 2 <= area.width / 3)
                    aprilTag = isRed ? 4 : 1;
                else if (foundPixel.x - area.x + foundPixel.width / 2 >= area.width * 2 / 3)
                    aprilTag = isRed ? 6 : 3;
                else
                    aprilTag = isRed ? 5 : 2;
                if (foundPixel.x - area.x + foundPixel.width / 2 <= area.width / 6)
                    position = Positions.LEFT;
                else if (foundPixel.x - area.x + foundPixel.width / 2 <= area.width * 2 / 6)
                    position = Positions.RIGHT;
                else if (foundPixel.x - area.x + foundPixel.width / 2 <= area.width * 3 / 6)
                    position = Positions.LEFT;
                else if (foundPixel.x - area.x + foundPixel.width / 2 <= area.width * 4 / 6)
                    position = Positions.RIGHT;
                else if (foundPixel.x - area.x + foundPixel.width / 2 <= area.width * 5 / 6)
                    position = Positions.LEFT;
                else if (foundPixel.x - area.x + foundPixel.width / 2 <= area.width)
                    position = Positions.RIGHT;
                canvas.drawRect(makeGraphicsRect(foundPixel, scaleBmpPxToCanvasPx), rectPaint);
            }
        }
        rectPaint.setColor(Color.GREEN);
        canvas.drawRect(makeGraphicsRect(area, scaleBmpPxToCanvasPx), rectPaint);
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }

    public Positions getPixelPosition() {
        return position;
    }

    public int getAprilTag() {
        return aprilTag;
    }

    boolean withinRange(double input1, double input2, double deviation)
    {
        return Math.abs(input1 - input2) <= deviation;
    }

    private void findBlue(ArrayList<MatOfPoint> contours, float scaleBmpPxToCanvasPx) {
        for(MatOfPoint c : contours) {
            double value = Imgproc.contourArea(c);
            if(value > treshold && value < limit) {
                Rect rect = Imgproc.boundingRect(c);
                if(rect.x < areaStart.x || rect.y < areaStart.y || rect.x + rect.width > areaEnd.x || rect.y + rect.height > areaEnd.y) continue;
                if (withinRange(rect.width * scaleBmpPxToCanvasPx, rect.height * scaleBmpPxToCanvasPx, maxHWDiff)) {
                    foundBluePixel = rect;
                }
            }
        }
    }
    private void findPurple(ArrayList<MatOfPoint> contours, float scaleBmpPxToCanvasPx) {
        for(MatOfPoint c : contours) {
            double value = Imgproc.contourArea(c);
            if(value > treshold && value < limit) {
                Rect rect = Imgproc.boundingRect(c);
                if(rect.x < areaStart.x || rect.y < areaStart.y || rect.x + rect.width > areaEnd.x || rect.y + rect.height > areaEnd.y) continue;
                if (withinRange(rect.width * scaleBmpPxToCanvasPx, rect.height * scaleBmpPxToCanvasPx, maxHWDiff)) {
                    foundPurplePixel = rect;
                }
            }
        }
    }
    private void findGreen(ArrayList<MatOfPoint> contours, float scaleBmpPxToCanvasPx) {
        for(MatOfPoint c : contours) {
            double value = Imgproc.contourArea(c);
            if(value > treshold && value < limit) {
                Rect rect = Imgproc.boundingRect(c);
                if(rect.x < areaStart.x || rect.y < areaStart.y || rect.x + rect.width > areaEnd.x || rect.y + rect.height > areaEnd.y) continue;
                if (withinRange(rect.width * scaleBmpPxToCanvasPx, rect.height * scaleBmpPxToCanvasPx, maxHWDiff)) {
                    foundGreenPixel = rect;
                }
            }
        }
    }
    private void findYellow(ArrayList<MatOfPoint> contours, float scaleBmpPxToCanvasPx) {
        for(MatOfPoint c : contours) {
            double value = Imgproc.contourArea(c);
            if(value > treshold && value < limit) {
                Rect rect = Imgproc.boundingRect(c);
                if(rect.x < areaStart.x || rect.y < areaStart.y || rect.x + rect.width > areaEnd.x || rect.y + rect.height > areaEnd.y) continue;
                if (withinRange(rect.width * scaleBmpPxToCanvasPx, rect.height * scaleBmpPxToCanvasPx, maxHWDiff)) {
                    foundYellowPixel = rect;
                }
            }
        }
    }
}

