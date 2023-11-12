package org.firstinspires.ftc.teamcode.subsystem.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class CSVisionProcessor implements VisionProcessor{
    private Rect rectLeft;
    private Rect rectMiddle;

    //THIS IS THE lower limit of the non view - i.e. saturation with nothing in it.
    private double defaultSaturationValue = 50;

    StartingPosition selection = StartingPosition.RIGHT;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    private static CSVisionProcessor _csVision;

    public static VisionProcessor getCSVision(
            int width,
            int leftX, int leftY,
            int middleX, int middleY
    ){
        _csVision = new CSVisionProcessor(width, leftX, leftY, middleX, middleY);
        return _csVision;
    }

    public static StartingPosition getPosition(){
        return _csVision.getStartingPosition();
    }

    public CSVisionProcessor(int width, int leftX, int leftY, int middleX, int middleY){
        rectLeft = new Rect(leftX, leftY,width, width);
        //overriding the middle rectangle size
        rectMiddle = new Rect(middleX, middleY, 310, 90);

    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);



        if ((satRectLeft > satRectMiddle) && (satRectLeft > defaultSaturationValue) ) {
            selection = StartingPosition.LEFT;

        }else if ((satRectMiddle > satRectLeft) && (satRectMiddle > defaultSaturationValue)){
            selection = StartingPosition.CENTER;

        }else{

            selection = StartingPosition.RIGHT;
        }

        return selection;
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    public double getDefaultSat(){
        return defaultSaturationValue;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float  scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);

        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);

        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelected = new Paint();
        nonSelected.setStrokeWidth(scaleCanvasDensity * 4);
        nonSelected.setStyle(Paint.Style.STROKE);
        nonSelected.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);


        selection = (StartingPosition) userContext;

        switch(selection){
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelected);

                break;

            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelected);
                canvas.drawRect(drawRectangleMiddle, nonSelected);

                break;
            case CENTER:
                canvas.drawRect(drawRectangleLeft, nonSelected);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);

                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelected);
                canvas.drawRect(drawRectangleMiddle, nonSelected);

                break;

        }

    }

    public StartingPosition getStartingPosition(){
        return selection;
    }

    public enum StartingPosition{
        NONE,
        LEFT,
        RIGHT,
        CENTER
    }

}
