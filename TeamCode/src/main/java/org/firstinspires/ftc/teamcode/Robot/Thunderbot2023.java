package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;


public class Thunderbot2023
{
    // defines all variables
    public IMU imu = null;

    Orientation angles = new Orientation();
    DcMotorEx leftFront = null;
    DcMotorEx rightFront = null;
    DcMotorEx leftRear = null;
    DcMotorEx rightRear = null;
    DistanceSensor lDistance = null;
    DistanceSensor rDistance = null;

    public LinearSlide linearSlide = new LinearSlide();
    public Delivery delivery = new Delivery();
    public EndGame endGame = new EndGame();
    public Intake intake = new Intake();
    public ArtemisEyes eyes = new ArtemisEyes();

    public Sensors sensors = new Sensors();
    List<LynxModule> allHubs;

    // Position Variables
    long leftFrontPosition = 0;
    long rightFrontPosition = 0;
    long leftRearPosition = 0;
    long rightRearPosition = 0;
    double allMotors = 0;
    double heading = 0;
    double initRotation = 0;
    double lastAngle = 0;
    double leftDistanceAway = 0;
    double rightDistanceAway = 0;

    boolean moving = false;

    public static double SPEED_GAIN = 0.05;
    public static double STRAFE_GAIN = 0.0075;
    public static double  TURN_GAIN = 0.001;
    public static double MAX_SPEED = 0.25;
    public static double MAX_STRAFE = 0.1;
    public static double MAX_TURN = 0.25;


    // converts inches to motor ticks
    static final double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    static final double DRIVE_GEAR_REDUCTION = 2.89 * 3.61;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    static final double WHEEL_DIAMETER_CM = 9.6;  // goBilda mecanum wheels are 96mm in diameter
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_CM * Math.PI);

    public static double MAX_VELOCITY_CM = 250;
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
    public Thunderbot2023() {}

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

        try
        {
            imu = ahwMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            lastAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            heading = lastAngle;
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("imu not found in config file", 0);
            imu = null;
        }

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
        try
        {
            rightFront = ahwMap.get(DcMotorEx.class, "rightFront");
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("rightFront not found in config file", 0);
        }

        try
        {
            rightRear = ahwMap.get(DcMotorEx.class, "rightRear");
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("rightRear not found in config file", 0);
        }

        try
        {
            leftFront = ahwMap.get(DcMotorEx.class, "leftFront");
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("leftFront not found in config file", 0);
        }

        try
        {
            leftRear = ahwMap.get(DcMotorEx.class, "leftRear");
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("leftRear not found in config file", 0);
        }
        try {
            lDistance = ahwMap.get(DistanceSensor.class, "lDistance");
        } catch (Exception e) {
            telemetry.addData("Left Distance Sensor not found", 0);
        }
        try {
            rDistance = ahwMap.get(DistanceSensor.class, "rDistance");
        } catch (Exception e) {
            telemetry.addData("Right Distance Sensor not found", 0);
        };

        try { linearSlide.init(ahwMap, telem); }
        catch(Exception e) { telemetry.addData("Lift not found", 0); }

        try {  endGame.init(ahwMap, telem); }
        catch(Exception e) { telemetry.addData("Drone Launcher not found", 0); }

        try { delivery.init(ahwMap, telem, ifAuto); }
        catch(Exception e) { telemetry.addData("Delivery not found", 0); }

        try {  intake.init(ahwMap, telem, ifAuto); }
        catch(Exception e) { telemetry.addData("Intake not found", 0); }

        try { sensors.init(ahwMap, telem); }
        catch (Exception e) {
            telemetry.addData("sensors not found", 0);
        }

    }

    /**
     * This code go's through the math behind the mecanum wheel drive.  Given the joystick values,
     * it will calculate the motor commands needed for the mecanum drive.
     *
     * @param forward    - Any forward motion including backwards
     * @param right     - Any movement from left to right
     * @param clockwise ffb - Any turning movements
     */
    public void joystickDrive(double forward, double right, double clockwise)
    {
        double frontLeft = forward + clockwise + right;
        double frontRight = forward - clockwise - right;
        double backLeft = forward + clockwise - right;
        double backRight = forward - clockwise + right;
        double max = abs(frontLeft);

        if (abs(frontRight) > max)
        {
            max = abs(frontRight);
        }
        if (abs(backLeft) > max)
        {
            max = abs(backLeft);
        }
        if (abs(backRight) > max)
        {
            max = abs(backRight);
        }
        if (max > 1)
        {
            frontLeft /= max;
            frontRight /= max;
            backLeft /= max;
            backRight /= max;
        }

        // From the REV Control Hub Docs...
        // https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback#using-run_using_encoder
        // In RUN_USING_ENCODER mode, you should set a velocity (measured in ticks per second),
        // rather than a power level. You can still provide a power level in RUN_USING_ENCODER mode,
        // but this is not recommended, as it will limit your target speed significantly.
        //
        // Therefore, we take the calculated values, frontLeft, frontRight, etc... and multiply them
        // by a maximum velocity for the chassis.
        leftFront.setVelocity( frontLeft * MAX_VELOCITY_TICKS );
        rightFront.setVelocity( frontRight * MAX_VELOCITY_TICKS );
        leftRear.setVelocity( backLeft * MAX_VELOCITY_TICKS );
        rightRear.setVelocity( backRight * MAX_VELOCITY_TICKS );
    }

    public void orientedDrive(double forward, double right, double clockwise) {
        double theta = toRadians(heading);
        double vx = (forward * cos(theta)) - (right * sin(theta));
        double vy = (forward * sin(theta)) + (right * cos(theta));

        vx *= 1.1;

        joystickDrive(vy, vx, clockwise);
    }
    public boolean alignToBackdrop(double targetDistance, double stickValue) {

        double avgDistance = (leftDistanceAway + rightDistanceAway)/2.0;
        double rangeError = (avgDistance - targetDistance);
        double headingError = leftDistanceAway - rightDistanceAway;

        if (rangeError < 1 && headingError < 0.5) {
            stop();
            moving = false;
            return true;
        }
        else {
            double y = Range.clip(-rangeError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
            double turn = Range.clip(-headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);

            joystickDrive(y, stickValue, turn);
            return false;
        }
    }
//    public boolean driveToTag(int tagID, double speed, double distanceAway)
//    {
//        if (!moving)
//        {
//            moving = true;
//        }
//
//        int tagNumber = eyes.getTagNumber(tagID);
//        double rangeError = (eyes.rangeError - distanceAway);
//        double headingError = eyes.headingError;
//        double yawError = eyes.yawError;
//
//        if (tagNumber == tagID) {
//
//            if (rangeError < 1 && headingError < 0.5 && yawError < 1) {
//                stop();
//                moving = false;
//                return true;
//            }
//            else {
//                double y = Range.clip(-rangeError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
//                double x = Range.clip(-yawError * STRAFE_GAIN, -MAX_STRAFE, MAX_STRAFE);
//                double turn = Range.clip(-headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);
//
//                joystickDrive(y, x, turn);
//                return false;
//            }
//        }
//        else {
//            stop();
//            return true;
//        }
//    }


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


    public boolean strafe(Direction dir, double distance, double power)
    {
        if ( dir == Direction.LEFT)
        {
            return drive(-90, distance,  power);
        }
        else
        {
            return drive( 90, distance, power);
        }
    }

    /**
     * Get the heading angle from the imu and convert it to degrees.
     * @return the heading angle
     */
    /**
     * Read the Z axis angle, accounting for the transition from +180 <-> -180.
     * Store the current angle in globalHeading.
     * Positive angles (+) are counterclockwise/CCW, negative angles (-) are clockwise/CW.
     * @return the current heading of the robot.
     */
    private double getHeading()
    {
        double rawImuAngle =  -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double delta = rawImuAngle - lastAngle;

        // An illustrative example: assume the robot is facing +179 degrees (last angle) and makes a +2 degree turn.
        // The raw IMU value will roll over from +180 to -180, so the final raw angle will be -179.
        // So delta = -179 - (+179) = -358.
        // Since delta is less than -180, add 360 to it: -358 + 360 = +2 (the amount we turned!)
        // This works the same way in the other direction.

        if(delta > 180)
        {
            delta -= 360;
        }
        else if(delta < -180)
        {
            delta += 360;
        }

        heading += delta; // change the global state
        lastAngle = rawImuAngle; // save the current raw Z state
        return heading;
    }


    public void update()
    {
        // Bulk data read.  MUST DO THIS EACH TIME THROUGH loop()
        for (LynxModule module : allHubs)
        {
            module.clearBulkCache();
        }

        leftFrontPosition = leftFront.getCurrentPosition();
        rightFrontPosition = rightFront.getCurrentPosition();
        leftRearPosition = leftRear.getCurrentPosition();
        rightRearPosition = rightRear.getCurrentPosition();

        allMotors = (double) (leftFrontPosition + rightFrontPosition + leftRearPosition + rightRearPosition) / 4;

        leftDistanceAway = lDistance.getDistance(DistanceUnit.CM);
        rightDistanceAway = rDistance.getDistance(DistanceUnit.CM);

        telemetry.addData("Motor Position", leftFrontPosition);
        telemetry.addData("Motor Powers:", leftFront.getVelocity());
        heading = getHeading();
        telemetry.addData("Heading: ", heading);

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
            if ( endGame != null ) { endGame.update();}
            if (linearSlide != null) {linearSlide.update(); }
        } catch (Exception e) {
            telemetry.addData("Exception in update() in Thunderbot2023 class.", 0);
        }
    }

    public void start(){}

    public void stop() {
        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftRear.setVelocity(0);
        rightRear.setVelocity(0);
    }

    public void resetIMUYaw() {
        imu.resetYaw();
    }

    public String getSpikePos() {
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

