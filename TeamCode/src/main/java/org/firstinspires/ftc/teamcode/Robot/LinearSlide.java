package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

@Config
public class LinearSlide
{

    public static boolean TELEM = false;
    Telemetry telemetry;

    // Defining Motors
    DcMotorEx leftLinear = null;
    DcMotorEx rightLinear = null;

    public double leftSlidePosition = 0;
    public double rightSlidePosition = 0;

    final private double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    final private double DRIVE_GEAR_REDUCTION = 3.61 * 5.23;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    final private double SPOOL_DIAMETER_CM = 3.5;  // slide spool is 35mm in diameter
    final private double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (SPOOL_DIAMETER_CM * Math.PI);
    // ^^^^ = 48.1 cm

    public static double SCALE_FACTOR = 1;
    public static double MIN_POS = 0;
    public static double MAX_POS = 36;
    public static double MAX_SPEED = 40;
    public static double DEFAULT_POWER = 1;
    public static double MAX_CURRENT_AMPS = 5.0;
    private int targetCounts;
    public int stepNumber = 0;

    public enum Positions
    {
        LEVEL_0(0),
        LEVEL_1(12),
        LEVEL_2(24),
        LEVEL_3(36);
        public final double cmHeight;

        Positions(double cm)
        {
            cmHeight = cm;
        }

    }

    // Initialize
    public void init(HardwareMap hwMap, Telemetry telem)
    {
        telemetry = telem;
        try
        {
            leftLinear = hwMap.get(DcMotorEx.class, "lSlide");
            leftLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLinear.setDirection(DcMotorSimple.Direction.FORWARD);
            leftLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Set a max current on teh linear slide so that we can detect if we are stalling
            // the motors from going too high or too low.
            leftLinear.setCurrentAlert(MAX_CURRENT_AMPS, AMPS);
        }
        catch (Exception e)
        {
            telemetry.addData("lSlide not found", 0);
        }
        try
        {
            rightLinear = hwMap.get(DcMotorEx.class, "rSlide");
            rightLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLinear.setDirection(DcMotorSimple.Direction.REVERSE);
            rightLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Set a max current on teh linear slide so that we can detect if we are stalling
            // the motors from going too high or too low.
            rightLinear.setCurrentAlert(MAX_CURRENT_AMPS, AMPS);

        }
        catch (Exception e)
        {
            telemetry.addData("rSlide not found", 0);
        }

    }

    public void linearPower(double power)
    {
        if (leftLinear != null)
        {
            leftLinear.setPower(power);
        }
        else
        {
            telemetry.addData("linear slide left motor not initialized.", 0);
        }
        if (rightLinear != null)
        {
            rightLinear.setPower(power);
        }
        else
        {
            telemetry.addData("linear slide right  motor not initialized.", 0);
        }
    }

    // Math behind the positions of the linear slides and seeing where it needs to stop and start

    public void goToLinear(Positions pos)
    {
        linearToPosition(pos.cmHeight, DEFAULT_POWER);
    }

    public double getLiftPosition()
    {
        return SCALE_FACTOR * (leftSlidePosition) / COUNTS_PER_CM;
    }


    public void linearToPosition(double cm, double power)
    {
        targetCounts = (int) (cm * COUNTS_PER_CM);
        if (leftLinear != null && rightLinear != null)
        {
            leftLinear.setTargetPosition(targetCounts);
            rightLinear.setTargetPosition(targetCounts);
            telemetry.addData("Target counts: ", targetCounts);
            leftLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Motor mode", leftLinear.getMode());
            leftLinear.setVelocity(power * MAX_SPEED * COUNTS_PER_CM);
            rightLinear.setVelocity(power * MAX_SPEED * COUNTS_PER_CM);
        }
        else
        {
            telemetry.addData("linear slide motor not initialized.", 0);
        }
    }

    public boolean isDone()
    {
        return leftLinear.isBusy() || rightLinear.isBusy();
    }

    public double toggleUp(double cmToggle)
    {
        double targetCM = getLiftPosition() + cmToggle;
        targetCM = Range.clip(targetCM, MIN_POS, MAX_POS);
        linearToPosition(targetCM, DEFAULT_POWER);
        return targetCM;
    }

    public double toggleDown(double cmToggle, boolean ignoreLimit)
    {
        double targetCM = getLiftPosition() - cmToggle;
        if (!ignoreLimit)
        {
            targetCM = Range.clip(targetCM, MIN_POS, MAX_POS);
        }
        linearToPosition(targetCM, DEFAULT_POWER);
        return targetCM;
    }

    public void slideTogglePositionsUp()
    {
        double position = getLiftPosition();
        if (position < Positions.LEVEL_1.cmHeight)
        {
            goToLinear(Positions.LEVEL_1);
        }
        else if (position < Positions.LEVEL_2.cmHeight)
        {
            goToLinear(Positions.LEVEL_2);
        }
        else if (position < Positions.LEVEL_3.cmHeight)
        {
            goToLinear(Positions.LEVEL_3);
        }
    }

    public void slideTogglePositionsDown()
    {
        double position = getLiftPosition();
        if (position > Positions.LEVEL_3.cmHeight)
        {
            goToLinear(Positions.LEVEL_3);
        }
        else if (position > Positions.LEVEL_2.cmHeight)
        {
            goToLinear(Positions.LEVEL_2);
        }
        else if (position > Positions.LEVEL_1.cmHeight)
        {
            goToLinear(Positions.LEVEL_1);
        }
        else
        {
            goToLinear(Positions.LEVEL_0);
        }
    }

    public void update()
    {
        if (leftLinear != null && rightLinear != null)
        {
            leftSlidePosition = leftLinear.getCurrentPosition();
            rightSlidePosition = rightLinear.getCurrentPosition();

            if (TELEM)
            {
                telemetry.addData("left Slide Position = ", leftSlidePosition);
                telemetry.addData("Right Slide Position = ", rightSlidePosition);
                telemetry.addData("Left Lift Current AMPS: ", leftLinear.getCurrent(AMPS));
                telemetry.addData("Right Lift Current AMPS: ", rightLinear.getCurrent(AMPS));
                telemetry.addData("Target Counts: ", targetCounts);
            }

            // TODO: Check to see if we are drawing too much current.  This could be caused by the
            //  linear slide motors going too low and bottoming out.
            //  CHECK THIS OVERCURRENT TEST. WHAT CURRENT INDICATES WE HAVE HIT BOTTOM?
//            if (leftLinear.isOverCurrent() || rightLinear.isOverCurrent() )
//            {
//                leftLinear.setVelocity(0);
//                rightLinear.setVelocity(0);
//                leftLinear.setTargetPosition( (int) leftSlidePosition);
//                rightLinear.setTargetPosition( (int) rightSlidePosition);
//            }
        }
        else
        {
            telemetry.addData("linear slide motors not initialized.", 0);
        }

        if (TELEM)
        {
            telemetry.addData("Slide Position = ", getLiftPosition());
            telemetry.addData("Left Slide Position: ", leftSlidePosition);
            telemetry.addData("Right Slide Position: ", rightSlidePosition);
            telemetry.addData("stepNumber", stepNumber);
        }
    }
}

