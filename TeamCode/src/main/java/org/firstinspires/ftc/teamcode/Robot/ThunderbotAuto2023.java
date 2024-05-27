package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

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
    double heading = 0;
    long leftFrontPosition = 0;
    long rightFrontPosition = 0;
    long leftRearPosition = 0;
    long rightRearPosition = 0;
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
     *
     * @param ahwMap
     * @param telem
     * @param withVision
     * @param ifAuto
     */
    public void init(HardwareMap ahwMap, Telemetry telem, boolean withVision, boolean ifAuto)
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

        try { delivery.init(ahwMap, telem, ifAuto); }
        catch(Exception e) { telemetry.addData("Delivery not found", 0); }

        try {  liftArms.init(ahwMap, telem); }
        catch(Exception e) { telemetry.addData("Drone Launcher not found", 0); }

        try {  intake.init(ahwMap, telem, ifAuto); }
        catch(Exception e) { telemetry.addData("Intake not found", 0); }

    }
    public void joystickDrive(double forward, double right, double clockwise) {
        PoseVelocity2d thePose = new PoseVelocity2d(new Vector2d(right, forward), clockwise);
        drive.setDrivePowers(thePose);
    }
    /**
     * Make the robot drive a certain distance in a certain direction.
     * @param targetHeading
     * @param distance
     * @param power
     * @return  boolean indicating true when the move is complete
     */
    public boolean drive(double targetHeading, double distance, double power)
    {


        double currentAngle = heading;

        // Set desired angle and initial distanceMoved
        if (!moving)
        {
            startAngle = heading;
//            initPosition = (long) allMotors;
            initPosition = leftFrontPosition;
            moving = true;
        }

        double distanceMoved;
        distanceMoved = abs(leftFrontPosition - initPosition);

        double distanceMovedInCM = distanceMoved / COUNTS_PER_CM;
        telemetry.addData("distanceMoved", distanceMoved);

        double currentPower = 0.1;

        if (distanceMovedInCM <= 0.1 * distance){
            currentPower += 0.00001;
            currentPower = Range.clip(currentPower, 0.1, 1.0);
        } else if (distanceMovedInCM > 0.9 * distance){
            currentPower -= 0.00001;
            currentPower = Range.clip(currentPower, 0.1, 1.0);
        } else {
            currentPower=power;
        }
        double xValue = Math.sin(toRadians(targetHeading)) * currentPower;
        double yValue = Math.cos(toRadians(targetHeading)) * currentPower;
        // calculates required speed to adjust to gyStartAngle
        double angleError = (startAngle - currentAngle) / 25;
        // Setting range of adjustments
        angleError = Range.clip(angleError, -1, 1);

        if (distanceMovedInCM >= distance)
        {
            // Stops when at the specified distance
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues if not at the specified distance
            telemetry.addData("y value", yValue);
            telemetry.addData("x value", xValue);
            telemetry.addData("angle error", angleError);

            joystickDrive(yValue, xValue, angleError);
            return false;
        }
    }

    /**
     * Drive the robot in the heading provided using the internal imu.  It will rotate to the heading
     * and tank drive along that heading.
     * @param targetHeading  the heading the robot should drive
     * @param distance the distance the robot should drive before stopping
     * @param power the speed the robot should drive
     * @return boolean indicating true when the move is complete
     */
    public boolean gyroDrive(double targetHeading, double distance, double power)
    {

        double currentAngle = heading;

        // Set desired angle and initial distanceMoved
        if (!moving)
        {
            startAngle = currentAngle;
            initPosition = (long) leftFrontPosition;
            moving = true;
        }

        double distanceMoved;

        distanceMoved = abs(leftFrontPosition - initPosition);

        double distanceMovedInCM = distanceMoved / COUNTS_PER_CM;

        double distanceRemaining = Math.abs(distanceMovedInCM - distance);

        if (distance > 30) {
            if (distanceRemaining < 30) {
                power = (distanceRemaining / 30) * power;
                if (power < 0) {
                    power = Range.clip(power, -1.0, -0.1);
                } else {
                    power = Range.clip(power, 0.1, 1.0);
                }

            }
        }

        telemetry.addData("distanceMoved", distanceMoved);

        // calculates required speed to adjust to gyStartAngle
        double angleError = (targetHeading - currentAngle) / 20;
        // Setting range of adjustments
        angleError = Range.clip(angleError, -1, 1);

        if (distanceMovedInCM >= distance)
        {
            // Stops when at the specified distance
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues if not at the specified distance
            joystickDrive(power, 0, angleError);
            telemetry.addData("power: ", power);
            telemetry.addData("Angle error", angleError);
            return false;
        }
    }

    /**
     * Turns the robot an amount of degrees using the imu
     *
     * @param degreesToTurn - Angle the robot will turn
     * @param power   - Speed the robot will turn
     *
     * @return boolean indicating true when the move is complete
     */
    public boolean turn(double degreesToTurn, double power)
    {
        // Updates current angle
        double currentAngle = heading;

        // Sets initial angle
        if (!moving)
        {
            startAngle = heading;
            moving = true;
        }

        power = abs(power);
        if (degreesToTurn < 0)
        {
            // Make power a negative number if the degreesTo to turn is negative
            power = -power;
        }

        if (Math.abs(currentAngle - startAngle) >= abs(degreesToTurn))
        {
            // Stops turning when at the specified angle
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues to turn if not at the specified angle
            joystickDrive(0, 0, power);
            return false;
        }
    }

    /**
     * Turns the robot to a face a desired heading
     * @param targetHeading
     * @param power
     * @return boolean indicating true when the move is complete
     */
    public boolean turnTo(double targetHeading, double power)
    {
        // Updates current angle
        double currentAngle = heading;

        if (!moving)
        {
            startAngle = currentAngle;
            moving = true;
        }

        double angleError = targetHeading - currentAngle;
        double angleErrorMagnitude = Math.abs(angleError);

        if (angleError < 0.0)
        {
            power *= -1.0;
        }

        // If the difference between the current angle and the target angle is small (<10), scale
        // the power proportionally to how far you have left to go.  But... don't let the power
        // get too small because the robot won't have enough power to complete the turn if the
        // power gets too small.
        if ( angleErrorMagnitude < 10)
        {
            power = power * angleErrorMagnitude / 50.0;

            if (power > 0)
            {
                power = Range.clip(power, 0.1, 1);
            }
            else
            {
                power = Range.clip(power, -1, -0.1);
            }
        }

        if ( angleErrorMagnitude <= 1.5)
        {
            // Stops turning when at the specified angle (or really close)
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues to turn if not at the specified angle
            joystickDrive(0, 0, power);
            telemetry.addData("power", power);
            telemetry.addData("angle error", angleError);
            telemetry.addData("target heading", targetHeading);
            return false;
        }
    }


    public boolean strafe(Thunderbot2023.Direction dir, double distance, double power)
    {
        if ( dir == Thunderbot2023.Direction.LEFT)
        {
            return drive(-90, distance,  power);
        }
        else
        {
            return drive( 90, distance, power);
        }
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

        // Need to move these to not require motor encoders since the Into the Deep season won't use them
        heading = drive.pose.heading.toDouble();
        leftFrontPosition = drive.leftFront.getCurrentPosition();
        leftRearPosition = drive.leftBack.getCurrentPosition();
        rightFrontPosition = drive.rightFront.getCurrentPosition();
        rightRearPosition = drive.rightBack.getCurrentPosition();

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

    public boolean blipDriver1() { return notifyTheDriver1; }
    public boolean blipDriver2() { return notifyTheDriver2; }


}

