package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
;

import java.util.List;


public class ThunderbotAuto2023
{

    public MecanumDrive drive;

    public LinearSlide linearSlide = new LinearSlide();
    public Delivery delivery = new Delivery();
    public LiftArms liftArms = new LiftArms();
    public Intake intake = new Intake();
    public ArtemisEyes eyes = new ArtemisEyes();
    List<LynxModule> allHubs;

    // Position Variables


    boolean moving = false;


    public  double MAX_SPEED = 0.25;
    public  double MAX_STRAFE = 0.1;
    public  double MAX_TURN = 0.15;


    // converts inches to motor ticks
    static final double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    static final double DRIVE_GEAR_REDUCTION = 2.89 * 3.61;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    static final double WHEEL_DIAMETER_CM = 9.6;  // goBilda mecanum wheels are 96mm in diameter
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_CM * Math.PI);

    public static double MAX_VELOCITY_CM = 200;
    static final double MAX_VELOCITY_TICKS = MAX_VELOCITY_CM * COUNTS_PER_CM;

    private Telemetry telemetry = null;
    private long initPosition;
    private double startAngle;
    private boolean notifyTheDriver1 = false;
    private boolean notifyTheDriver2 = false;

    public enum Direction
    {
        LEFT,
        RIGHT;
    }

    /**
     * Constructor
     */
    public ThunderbotAuto2023() {}

    /**
     * Initializes the Thunderbot and connects its hardware to the HardwareMap
     * @param ahwMap
     * @param telem
     * @param withVision
     */
    public void init(HardwareMap ahwMap, Telemetry telem, boolean withVision)
    {
        telemetry = telem;

        drive = new MecanumDrive(ahwMap, new Pose2d(0,0,0));

        try {
            allHubs = ahwMap.getAll(LynxModule.class);

            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
        catch (Exception e) {
            telemetry.addData("Lynx Module not initialized", 0);
        }

        if ( withVision )
        {
            try
            {
                eyes.init(ahwMap, telem);
            }
            catch (Exception e)
            {
                telemetry.addData("Vision Processor not initialized.", 0);
            }
        }
        // Define & Initialize Motors


        try { linearSlide.init(ahwMap, telem); }
        catch(Exception e) { telemetry.addData("Lift not found", 0); }

        try { delivery.init(ahwMap, telem, true); }
        catch(Exception e) { telemetry.addData("Delivery not found", 0); }

        try {  liftArms.init(ahwMap, telem); }
        catch(Exception e) { telemetry.addData("Drone Launcher not found", 0); }

        try {  intake.init(ahwMap, telem, true); }
        catch(Exception e) { telemetry.addData("Intake not found", 0); }

    }
    public void joystickDrive(double forward, double right, double clockwise) {
        PoseVelocity2d thePose = new PoseVelocity2d(new Vector2d(right, forward), clockwise);
        drive.setDrivePowers(thePose);
    }

    public boolean driveToTag(int tagID, double speed, double distanceAway)
    {
        double SPEED_GAIN = 0.0075;
        double STRAFE_GAIN = 0.0075;
        double  TURN_GAIN = 0.001;
        if (!moving)
        {
            moving = true;
        }

        int tagNumber = eyes.getTagNumber(tagID);
        double rangeError = (eyes.rangeError - distanceAway);
        double headingError = eyes.headingError;
        double yawError = eyes.yawError;

        if (tagNumber == tagID) {

            if (rangeError < 1 && headingError < 0.5 && yawError < 1) {
                stop();
                moving = false;
                return true;
            }
            else {
                double y = Range.clip(-rangeError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
                double x = Range.clip(-yawError * STRAFE_GAIN, -MAX_STRAFE, MAX_STRAFE);
                double turn = Range.clip(-headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);

                joystickDrive(y, x, turn);
                return false;
            }
        }
        else {
            stop();
            return true;
        }
    }

    public void update()
    {
        // Bulk data read.  MUST DO THIS EACH TIME THROUGH loop()
        for (LynxModule module : allHubs)
        {
            module.clearBulkCache();
        }
        drive.updatePoseEstimate();

        telemetry.addData("Motor Position", drive.pose);
        telemetry.addData("Heading: ", drive.pose.heading);

        notifyTheDriver1 = false;
        notifyTheDriver2 = false;
        try {
            if ( intake != null ) {
                intake.update();
                notifyTheDriver1 = intake.gripperClosed();
            }
            if ( delivery != null ) {
                delivery.update();
                notifyTheDriver2 = delivery.gripperClosed();
            }
            if ( liftArms != null ) { liftArms.update();}
            if (linearSlide != null) {linearSlide.update(); }
        } catch (Exception e) {
            telemetry.addData("Exception in update() in Thunderbot2023 class.", 0);
        }
    }

    public void start(){}

    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    public String getSpikePos()
    {
        if ( eyes != null ) { return eyes.getSpikePos(); }
        else { return "No Vision System Initialized."; }
    }
    public double getPropX()
    {
        if (eyes != null) {  return eyes.getPropX(); }
        else { return -1; }
    }
    public double getPropY()
    {
        if (eyes != null) { return eyes.getPropY(); }
        else  { return -1; }
    }

    public boolean notifyDriver1() { return notifyTheDriver1; }
    public boolean notifyDriver2() { return notifyTheDriver2; }


}

