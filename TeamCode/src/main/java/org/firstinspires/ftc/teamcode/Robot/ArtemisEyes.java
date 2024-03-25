package org.firstinspires.ftc.teamcode.Robot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

public class ArtemisEyes
{
    private WebcamName theFrontCamera;
    private WebcamName theBackCamera;
    public TGEVisionProcessor tgeFinder;
    AprilTagProcessor aprilTagFinder;
    VisionPortal thePortal;
    Telemetry telemetry;
    double rangeError;
    double headingError;
    double yawError;
    void init(HardwareMap hardwareMap, Telemetry telem )
    {
        telemetry = telem;
        try
        {
            theFrontCamera = hardwareMap.get(WebcamName.class, "Webcam 1");

        }
        catch (Exception e)
        {
            telemetry.addData("Webcam 1 not found -- theFrontCamera", 0);
        }
        try
        {
            theBackCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        }
        catch (Exception e)
        {
            telemetry.addData("Webcam 2 not found -- theRearCamera", 0);
        }

        try
        {
            if ( theFrontCamera != null && theBackCamera != null )
            {
                CameraName switchableCamera = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(
                        theFrontCamera, theBackCamera);
                aprilTagFinder = new AprilTagProcessor.Builder().setDrawTagOutline(true).build();
                tgeFinder = new TGEVisionProcessor();
                thePortal = VisionPortal.easyCreateWithDefaults(switchableCamera, aprilTagFinder, tgeFinder);
                aprilTagFinder.setDecimation(2);
               // thePortal.setActiveCamera(theFrontCamera);
            }
            else if ( theBackCamera == null)
            {
                aprilTagFinder = new AprilTagProcessor.Builder().setDrawTagOutline(true).build();
                aprilTagFinder.setDecimation(2);

                tgeFinder = new TGEVisionProcessor();
                tgeFinder.setTelemetry(telemetry);

                thePortal = new VisionPortal.Builder()
                        .setCamera(theFrontCamera)
                        .addProcessors(aprilTagFinder, tgeFinder)
                        .build();
//                thePortal = VisionPortal.easyCreateWithDefaults(theFrontCamera, aprilTagFinder, tgeFinder);
               // thePortal.setActiveCamera(theFrontCamera);
            }
            else
            {
                aprilTagFinder = new AprilTagProcessor.Builder().setDrawTagOutline(true).build();
                tgeFinder = new TGEVisionProcessor();
                tgeFinder.setTelemetry(telemetry);
                thePortal = VisionPortal.easyCreateWithDefaults(theBackCamera, aprilTagFinder, tgeFinder);
                aprilTagFinder.setDecimation(2);
                //thePortal.setActiveCamera(theBackCamera);
            }
            thePortal.stopLiveView();
        }
        catch (Exception e)
        {
            telemetry.addData("exception:  ", e.getMessage());
        }


    }

    public String getSpikePos() {
        if (tgeFinder != null) {
            if (thePortal.getProcessorEnabled(tgeFinder)){
                telemetry.addData("TGE Enabled YES!!", 0);
            }
            return tgeFinder.getSpikePos();

        } else {
            return "TGEFINDER NOT INITIALIZED";
        }


    }

    @SuppressLint("DefaultLocale")
    public int getTagNumber(int tagNum)
    {
        List<AprilTagDetection> currentDetections = aprilTagFinder.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        int tagPos = -1;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.metadata != null && detection.id == tagNum) {
                rangeError = detection.ftcPose.range;
                headingError = detection.ftcPose.bearing;
                yawError = detection.ftcPose.yaw;

                tagPos = detection.id;
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else if (detection.metadata == null) {
                telemetry.addLine(String.format("No Detections Found"));
            }
            else
            {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        return tagPos;
    }

    public double getPropX() {
        if (tgeFinder != null) {
            return tgeFinder.xPos;
        } else {
            return -1;
        }
    }
    public double getPropY() {
        if (tgeFinder != null) {
            return tgeFinder.yPos;
        } else {
            return -1;
        }
    }

    public void activateFrontCamera()
    {
        if ( theFrontCamera != null )
        {
            thePortal.setActiveCamera(theFrontCamera);
        }
        else {
            telemetry.addData("Can't activate front camera. Front camera not initialized.", 0);
        }

    }
    public void activateBackCamera()
    {
        if ( theBackCamera != null )
        {
            thePortal.setActiveCamera(theBackCamera);
        }
        else {
            telemetry.addData("Can't activate back camera. Back camera not initialized.", 0);
        }
    }

    public void stopPropVisionProcessor()
    {
        if ( tgeFinder != null )
        {
            thePortal.setProcessorEnabled(tgeFinder, false);
        }
        else {
            telemetry.addData("Can't disable Prop Vision Processor. Not initialized.", 0);
        }
    }

    public void stopAprilTagProcessor()
    {
        if ( aprilTagFinder != null )
        {
            thePortal.setProcessorEnabled(aprilTagFinder, false);
        }
        else {
            telemetry.addData("Can't disable AprilTag Processor. Not initialized.", 0);
        }
    }
    public void startPropVisionProcessor()
    {
        if ( tgeFinder != null )
        {
            thePortal.setProcessorEnabled(tgeFinder, true);
        }
        else {
            telemetry.addData("Can't enable Prop Vision Processor. Not initialized.", 0);
        }
    }

    public void startAprilTagProcessor()
    {
        if ( aprilTagFinder != null )
        {
            thePortal.setProcessorEnabled(aprilTagFinder, true);
        }
        else {
            telemetry.addData("Can't enable AprilTag Processor. Not initialized.", 0);
        }
    }

}
