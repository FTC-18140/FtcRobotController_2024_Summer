package org.firstinspires.ftc.teamcode.Robot;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.graphics.Rect;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.imgproc.Moments;


import java.util.ArrayList;
import java.util.List;

import static android.graphics.Bitmap.createBitmap;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TGEVisionProcessor implements VisionProcessor
{

    //Outputs
    private  Mat cvCvtcolorOutput = new Mat();
    private  Mat cvExtractchannelOutput = new Mat();
    private  Mat cvThresholdOutput = new Mat();
    private  Mat cvDilate0Output = new Mat();
    private  Mat cvErodeOutput = new Mat();
    private  Mat cvDilate1Output = new Mat();

    Mat cvCvtcolorSrc;
    Mat cvExtractchannelSrc;
    Mat cvThresholdSrc;
    Mat cvDilate0Src;
    Mat cvErodeSrc;
    Mat cvDilate1Src;
    Mat findContoursInput;
    Bitmap myBitmap;
    Mat myMat;

    double inittime = 0;
    int index = -1;

    Point [] contourPts;
    long numPts = 0;

    private final ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private String spikePos = "LEFT";

    double xPos = 320;
    double yPos = 240;

    String bitmapText = "";

    int numCountours = 0;

    public static int minThreshold = 140;
    public static int maxThreshold = 255;

    public static int erodeSize = 20;

    public static String theColor = "RED";
    public static int extractChannel = 1;

    public void setTelemetry(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    Telemetry telemetry;

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        inittime = System.nanoTime() / 1_000_000_000.0;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        if (theColor.equalsIgnoreCase("RED"))
        {
            minThreshold = 140;
            extractChannel = 1;
        }
        else
        {
            minThreshold = 170;
            extractChannel = 2;
        }

        // Step CV_cvtColor0:
        cvCvtcolorSrc = frame;
        int cvCvtcolorCode = Imgproc.COLOR_RGB2YCrCb;
        cvCvtcolor(cvCvtcolorSrc, cvCvtcolorCode, cvCvtcolorOutput);

        // Step CV_extractChannel0:
        cvExtractchannelSrc = cvCvtcolorOutput;
        double cvExtractchannelChannel = extractChannel;
        cvExtractchannel(cvExtractchannelSrc, cvExtractchannelChannel, cvExtractchannelOutput);

        // Step CV_Threshold0:
        cvThresholdSrc = cvExtractchannelOutput;
        double cvThresholdThresh = minThreshold;
        double cvThresholdMaxval = maxThreshold;
        int cvThresholdType = Imgproc.THRESH_BINARY;
        cvThreshold(cvThresholdSrc, cvThresholdThresh, cvThresholdMaxval, cvThresholdType, cvThresholdOutput);

        // Step CV_dilate0:
        cvDilate0Src = cvThresholdOutput;
        Mat cvDilate0Kernel = new Mat();
        Point cvDilate0Anchor = new Point(-1, -1);
        double cvDilate0Iterations = 1.0;
        int cvDilate0Bordertype = Core.BORDER_CONSTANT;
        Scalar cvDilate0Bordervalue = new Scalar(-1);
        cvDilate(cvDilate0Src, cvDilate0Kernel, cvDilate0Anchor, cvDilate0Iterations, cvDilate0Bordertype, cvDilate0Bordervalue, cvDilate0Output);

        // Step CV_erode0:
        cvErodeSrc = cvDilate0Output;
        Mat cvErodeKernel = new Mat();
        Point cvErodeAnchor = new Point(-1, -1);
        double cvErodeIterations = erodeSize;
        int cvErodeBordertype = Core.BORDER_CONSTANT;
        Scalar cvErodeBordervalue = new Scalar(-1);
        cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

        // Step CV_dilate1:
        cvDilate1Src = cvErodeOutput;
        Mat cvDilate1Kernel = new Mat();
        Point cvDilate1Anchor = new Point(-1, -1);
        double cvDilate1Iterations = erodeSize;
        int cvDilate1Bordertype = Core.BORDER_CONSTANT;
        Scalar cvDilate1Bordervalue = new Scalar(-1);
        cvDilate(cvDilate1Src, cvDilate1Kernel, cvDilate1Anchor, cvDilate1Iterations, cvDilate1Bordertype, cvDilate1Bordervalue, cvDilate1Output);

        // Step Find_Contours0:
        findContoursInput = cvDilate1Output;
        boolean findContoursExternalOnly = false;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

        numCountours = findContoursOutput.size();

        if ( numCountours > 0 )
        {
            double maxArea = 0;
            int maxIndex = 0;

            for ( int i = 0; i < findContoursOutput.size(); i++ )
            {
                double currentArea = Imgproc.contourArea( findContoursOutput.get(i) );
                if ( currentArea > maxArea)
                {
                    maxArea = currentArea;
                    maxIndex = i;
                }
            }
            contourPts = findContoursOutput.get(maxIndex).toArray();
            numPts = findContoursOutput.get(maxIndex).total();

            // Calculate moments of the contour
            Moments moments = Imgproc.moments(findContoursOutput.get(maxIndex));

            // find X and Y of the centroid of the contour
            xPos = moments.m10/moments.m00;
            yPos = moments.m01/moments.m00;

            if (xPos > 200 )
            {
                spikePos = "RIGHT";// RIGHT
            }
            else if ( xPos < 200)
            {
                spikePos = "CENTER";   // CENTER
            }
            else
            {
                spikePos = "LEFT";// LEFT
            }
        }
        else
        {
            spikePos = "NOT FOUND";// CENTER
        }
        return frame;
    }

    public String getSpikePos()
    {
        return spikePos;
    }



    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        if ((System.nanoTime() / 1_000_000_000.0) - inittime > 1)
        {
            index++;
            inittime = System.nanoTime() / 1_000_000_000.0;
        }

        switch(index)
        {
            case 0:
                myMat = cvCvtcolorOutput();
                bitmapText = "New Color";
                break;
            case 1:
                myMat = cvExtractchannelOutput();
                bitmapText = "Extract Channel";
                break;
            case 2:
                myMat = cvThresholdOutput();
                bitmapText = "Threshold";
                break;
            case 3:
                myMat = cvDilate0Output();
                bitmapText = "Dilate";
                break;
            case 4:
                myMat = cvErodeOutput();
                bitmapText = "Erode";
                break;
            case 5:
                myMat = cvDilate1Output();
                bitmapText = "Dilate Again";
                break;
            default:
                index = 0;
                break;
        }


        try
        {
            myBitmap = createBitmap( myMat.width(), myMat.height(), Bitmap.Config.ARGB_8888);

            Utils.matToBitmap(myMat, myBitmap);
            Rect drawRegion = new Rect(0, 0, onscreenWidth, onscreenHeight);
            canvas.drawBitmap(myBitmap, null, drawRegion, null);

            Paint bitmapTextPaint = new Paint();
            bitmapTextPaint.setColor(Color.YELLOW);
            bitmapTextPaint.setAntiAlias(true);
            bitmapTextPaint.setTypeface(Typeface.DEFAULT_BOLD);
            bitmapTextPaint.setTextSize(50);
            canvas.drawText( bitmapText, 400, 600, bitmapTextPaint );

            Paint textPaint = new Paint();
            textPaint.setColor(Color.YELLOW);
            textPaint.setAntiAlias(true);
            textPaint.setTypeface(Typeface.DEFAULT_BOLD);
            textPaint.setTextSize(50);
            canvas.drawText( spikePos, 100, 450, textPaint );

            Paint circlePaint = new Paint();
            circlePaint.setColor(Color.CYAN);
            circlePaint.setStrokeWidth(5);
            canvas.drawCircle( (float) xPos, (float) yPos, 25, circlePaint);

            Paint contourPaint = new Paint();
            contourPaint.setColor(Color.MAGENTA);
            contourPaint.setStrokeWidth(10);

            if ( contourPts != null )
            {
                for (int j = 0; j < numPts; j++)
                {
                    canvas.drawPoint((float) contourPts[j].x, (float) contourPts[j].y,
                                     contourPaint);
                }
            }

        }
        catch (Exception e)
        {
            telemetry.addData("myMat: ", bitmapText);
            telemetry.update();
        }

//        canvas.drawText( Integer.toString(cvCvtcolorOutput.width()), 300, 300, textPaint );
//        canvas.drawText( Integer.toString(cvCvtcolorOutput.height()), 500, 300, textPaint );


    }

    /**
     * This method is a generated getter for the output of a CV_cvtColor.
     * @return Mat output from CV_cvtColor.
     */
    public Mat cvCvtcolorOutput() {
        return cvCvtcolorOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_extractChannel.
     * @return Mat output from CV_extractChannel.
     */
    public Mat cvExtractchannelOutput() {
        return cvExtractchannelOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_Threshold.
     * @return Mat output from CV_Threshold.
     */
    public Mat cvThresholdOutput() {
        return cvThresholdOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_dilate.
     * @return Mat output from CV_dilate.
     */
    public Mat cvDilate0Output() {
        return cvDilate0Output;
    }

    /**
     * This method is a generated getter for the output of a CV_erode.
     * @return Mat output from CV_erode.
     */
    public Mat cvErodeOutput() {
        return cvErodeOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_dilate.
     * @return Mat output from CV_dilate.
     */
    public Mat cvDilate1Output() {
        return cvDilate1Output;
    }

    /**
     * This method is a generated getter for the output of a Find_Contours.
     * @return ArrayList<MatOfPoint> output from Find_Contours.
     */
    public ArrayList<MatOfPoint> findContoursOutput() {
        return findContoursOutput;
    }


    /**
     * Converts an image from one color space to another.
     * @param src Image to convert.
     * @param code conversion code.
     * @param dst converted Image.
     */
    private void cvCvtcolor(Mat src, int code, Mat dst) {
        Imgproc.cvtColor(src, dst, code);
    }

    /**
     * Extracts given channel from an image.
     * @param src the image to extract.
     * @param channel zero indexed channel number to extract.
     * @param dst output image.
     */
    private void cvExtractchannel(Mat src, double channel, Mat dst) {
        Core.extractChannel(src, dst, (int)channel);
    }

    /**
     * Apply a fixed-level threshold to each array element in an image.
     * @param src Image to threshold.
     * @param threshold threshold value.
     * @param maxVal Maximum value for THRES_BINARY and THRES_BINARY_INV
     * @param type Type of threshold to appy.
     * @param dst output Image.
     */
    private void cvThreshold(Mat src, double threshold, double maxVal, int type,
                             Mat dst) {
        Imgproc.threshold(src, dst, threshold, maxVal, type);
    }

    /**
     * Expands area of lower value in an image.
     * @param src the Image to erode.
     * @param kernel the kernel for erosion.
     * @param anchor the center of the kernel.
     * @param iterations the number of times to perform the erosion.
     * @param borderType pixel extrapolation method.
     * @param borderValue value to be used for a constant border.
     * @param dst Output Image.
     */
    private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                         int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null) {
            borderValue = new Scalar(-1);
        }
        Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    /**
     * Expands area of higher value in an image.
     * @param src the Image to dilate.
     * @param kernel the kernel for dilation.
     * @param anchor the center of the kernel.
     * @param iterations the number of times to perform the dilation.
     * @param borderType pixel extrapolation method.
     * @param borderValue value to be used for a constant border.
     * @param dst Output Image.
     */
    private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
                          int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null){
            borderValue = new Scalar(-1);
        }
        Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }


    private void findContours(Mat input, boolean externalOnly,
                              List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

}
