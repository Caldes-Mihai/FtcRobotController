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
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class PropProcessor implements VisionProcessor {

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
    public Scalar lower2 = new Scalar(30, 93, 0);
    public Scalar upper2 = new Scalar(100, 255, 255);
    public Scalar lower = new Scalar(0, 0, 135);
    public Scalar upper = new Scalar(162, 39, 255);
    public Point A = new Point(100, 100);
    public Point B = new Point(500, 300);
    public int range = 1000;
    public boolean isRed;
    public enum Positions {
        LEFT,
        CENTER,
        RIGHT
    }

    public static Positions position = Positions.LEFT;
    int maxWProp = 0;
    int maxHProp = 0;
    Rect foundProp;
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
    private Mat propMat = new Mat();
    private Telemetry t;
    /**
     * Enum to choose which color space to choose
     * with the live variable tuner isntead of
     * hardcoding it.
     */

    public PropProcessor(Telemetry t, boolean isRed) {
        this.t = t;
        this.isRed = isRed;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, propMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(propMat, isRed ? lower : lower2, isRed ? upper : upper2, propMat);
        List<MatOfPoint> propContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        try {
            Imgproc.findContours(propMat, propContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        } catch (Exception e) {}
        propMat.release();
        return propContours;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(isRed ? Color.RED : Color.BLUE);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
        ArrayList<MatOfPoint> propContours = (ArrayList) userContext;
        maxWProp = 0;
        maxHProp = 0;
        foundProp = null;
        for(MatOfPoint c : propContours) {
            double value = Imgproc.contourArea(c);
            if(value > 100) {
                Rect rect = Imgproc.boundingRect(c);
                if(rect.x < A.x || rect.y < A.y || rect.x + rect.width > B.x || rect.y + rect.height > B.y) continue;
                if ((rect.width > maxWProp || rect.height > maxHProp) && withinRange(rect.width * scaleBmpPxToCanvasPx, rect.height * scaleBmpPxToCanvasPx, range)) {
                    maxWProp = rect.width;
                    maxHProp = rect.height;
                    foundProp = rect;
                }
            }
        }
        if(foundProp != null) {
            if (foundProp.x * scaleBmpPxToCanvasPx <= onscreenWidth / 3)
                position = Positions.RIGHT;
            else if (foundProp.x * scaleBmpPxToCanvasPx >= onscreenWidth * 2 / 3)
                position = position.LEFT;
            else
                position = position.CENTER;
            canvas.drawRect(makeGraphicsRect(foundProp, scaleBmpPxToCanvasPx), rectPaint);
        }
        rectPaint.setColor(Color.GREEN);
        canvas.drawRect(makeGraphicsRect(new Rect(A, B), scaleBmpPxToCanvasPx), rectPaint);
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }

    public Positions getPropPosition() {
        return position;
    }

    boolean withinRange(double input1, double input2, double deviation)
    {
        return Math.abs(input1 - input2) <= deviation;
    }
}

