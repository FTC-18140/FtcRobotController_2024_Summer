package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.util.Range.clip;

@Config
public class Delivery
{
    public static boolean TELEM = false;
    Telemetry telemetry;

    // All the Attachment Defining
    public Servo wrist = null;
    public Servo leftGripper = null;
    public Servo rightGripper = null;
    public Servo twist = null;
    public Servo lElbow = null;
    public Servo rElbow = null;

    private boolean leftGripperClosed = false;
    private boolean rightGripperClosed = false;
    public double wristPos = 0;
    public double leftGripPos = 0.05;
    public double rightGripPos = 0.5;
    public double twistPos = 0.5;
    public double lElbowPos = 0.92;
    public double rElbowPos = 0.92;

    static public double ELBOW_MIN = 0.775;
    // MIN is whenever the elbow is completely up ready to receive
    static public double ELBOW_MAX = 0.92;
    // MAX is whenever the elbow is down
    static public double WRIST_MIN = 0.31;
    static public double WRIST_MAX = 0.92;

    static public double ELBOW_INIT = 0.92;
    static public double WRIST_INIT = 0.92;
    static public double LEFTGRIP_INIT = 0.25;
    static public double RIGHTGRIP_INIT = 0.75;
    static public double TWIST_INIT = 0.5;
    static public double LEFT_GRIP_DROP = 0.5;
    static public double RIGHT_GRIP_DROP = 0.5;
    //Initalization should be 0.46
    // 0.225 is the position to get ready to pick up
    static public double TWIST_TOGGLE_INCREMENT = 15;
    private double WRIST_TOGGLE_INCREMENT = 10;

    private boolean clearOfTransferZone = false;


    public enum Positions
    {
        READY_TO_TRANSFER(ELBOW_MIN, ELBOW_MIN, WRIST_INIT, TWIST_INIT, GripperPositions.RELEASED),
        TRANSFER(0, 0, 0, 0, GripperPositions.GRIPPED),
        TELE_INIT(ELBOW_MAX, ELBOW_MAX, WRIST_INIT, TWIST_INIT, GripperPositions.RELEASED),
        AUTO_INIT(ELBOW_INIT, ELBOW_INIT, WRIST_INIT, TWIST_INIT, GripperPositions.GRIPPED),
        ALIGN_FOR_TRANSFER(0.92, 0.92, 0.92, 0.5, GripperPositions.RELEASED),
        ALIGN_TO_BACKDROP(0.81, 0.81, 0.775, 0.5, GripperPositions.GRIPPED);

        public final double lElbowPos;
        public final double rElbowPos;
        public final double wristPos;
        public final double twisterPos;
        public final GripperPositions grip;

        Positions(double lElbow, double rElbow, double wrist, double twist, GripperPositions pos)
        {
            lElbowPos = lElbow;
            rElbowPos = rElbow;
            wristPos = wrist;
            twisterPos = twist;
            grip = pos;
        }
    }

    public enum GripperPositions
    {
        // IF ANY GRIP ISSUES CHANGE THE GRIP POSITIONS TO 0.8 INSTEAD OF 0.775 (ITS TIGHTER)
        GRIPPED(0.85, 0.85),
        RELEASED(0.5, 0.5),
        INIT(LEFTGRIP_INIT, RIGHTGRIP_INIT);

        public final double leftGripPos;
        public final double rightGripPos;

        GripperPositions(double leftGrip, double rightGrip)
        {
            leftGripPos = leftGrip;
            rightGripPos = rightGrip;
        }
    }

    public void init(HardwareMap hwMap, Telemetry telem, boolean ifAuto)
    {
        telemetry = telem;

        try
        {
            wrist = hwMap.servo.get("wrist");
        }
        catch (Exception e)
        {
            telemetry.addData("wrist did not initialize", 0);
        }

        try
        {
            leftGripper = hwMap.servo.get("leftGrip");
            leftGripper.setDirection(Servo.Direction.FORWARD);
        }
        catch (Exception e)
        {
            telemetry.addData("leftGrip did not initialize", 0);
        }

        try
        {
            rightGripper = hwMap.servo.get("rightGrip");
            rightGripper.setDirection(Servo.Direction.REVERSE);
        }
        catch (Exception e)
        {
            telemetry.addData("rightGrip did not initialize", 0);
        }
        try
        {
            twist = hwMap.servo.get("twist");
            twist.setDirection(Servo.Direction.REVERSE);
        }
        catch (Exception e)
        {
            telemetry.addData("twist did not initialize", 0);
        }

        try
        {
            lElbow = hwMap.servo.get("lElbow");
            lElbow.setDirection(Servo.Direction.FORWARD);

        }
        catch (Exception e)
        {
            telemetry.addData("lElbow did not initialize", 0);
        }

        try
        {
            rElbow = hwMap.servo.get("rElbow");
            rElbow.setDirection(Servo.Direction.REVERSE);
        }
        catch (Exception e)
        {
            telemetry.addData("rElbow did not initialize", 0);
        }

        // Now that everything is inited, put the Delivery into a good position.
        if (ifAuto)
        {
            goTo(Positions.AUTO_INIT);
        }
        else
        {
            goTo(Positions.TELE_INIT);
        }
    }

    public double setWristPosition(double wristPos)
    {
        if (wrist != null)
        {
            double clippedPos = clip(wristPos, WRIST_MIN, WRIST_MAX);
            wrist.setPosition(clippedPos);
            return clippedPos;
        }
        else
        {
            telemetry.addData("delivery wrist not initialized.", 0);
        }
        return -1;
    }

    private boolean autoSetWristPos(double position)
    {
        wrist.setPosition(position);
        return true;
    }

    public void setLeftGripPos(double leftGripPos)
    {
        if (leftGripper != null)
        {
            leftGripper.setPosition(leftGripPos);
        }
        else
        {
            telemetry.addData("delivery left gripper not initialized.", 0);
        }
    }

    public void setRightGripPos(double rightGripPos)
    {
        if (rightGripper != null)
        {
            rightGripper.setPosition(rightGripPos);
        }
        else
        {
            telemetry.addData("delivery right gripper not initialized.", 0);
        }
    }

    public void setTwistPos(double twistPos)
    {
        if (twist != null)
        {
            double clippedPos = clip(twistPos, 0, 1);
            twist.setPosition(clippedPos);
        }
        else
        {
            telemetry.addData("delivery twist not initialized.", 0);
        }
    }

    public double setElbowPosition(double position)
    {
        double clippedPosition = clip(position, ELBOW_MIN, ELBOW_MAX);
        setlElbowPos(clippedPosition);
        setrElbowPos(clippedPosition);
        return clippedPosition;
    }

    private boolean setElbowPos(double position)
    {
        lElbow.setPosition(position);
        rElbow.setPosition(position);
        return true;
    }

    public void setlElbowPos(double lElbowPos)
    {
        if (lElbow != null)
        {
            lElbow.setPosition(lElbowPos);
        }
        else
        {
            telemetry.addData("delivery left elbow not initialized.", 0);
        }
    }

    public void setrElbowPos(double rElbowPos)
    {
        if (rElbow != null)
        {
            rElbow.setPosition(rElbowPos);
        }
        else
        {
            telemetry.addData("delivery right elbow not initialized.", 0);
        }
    }

    public void dropBoth()
    {
        dropLeft();
        dropRight();
    }

    public void dropLeft()
    {
        setLeftGripPos(GripperPositions.RELEASED.leftGripPos);
    }

    public void dropRight()
    {
        setRightGripPos(GripperPositions.RELEASED.rightGripPos);
    }

    public void holdPixelsBoth()
    {
        holdPixelLeft();
        holdPixelRight();
    }

    public void holdPixelLeft()
    {
        setLeftGripPos(GripperPositions.GRIPPED.leftGripPos);
    }

    public void holdPixelRight()
    {
        setRightGripPos(GripperPositions.GRIPPED.rightGripPos);
    }

    public void goTo(Positions thePos)
    {
        setlElbowPos(thePos.lElbowPos);
        setrElbowPos(thePos.lElbowPos);
        setWristPosition(thePos.wristPos);
        setTwistPos(thePos.twisterPos);
        setRightGripPos(thePos.grip.rightGripPos);
        setLeftGripPos(thePos.grip.leftGripPos);
    }

    public void toggleUp()
    {
        double newWristPos = wristPos - (WRIST_TOGGLE_INCREMENT / 180.0);
        setWristPosition(newWristPos);
    }

    public void toggleDown()
    {
        double newWristPos = wristPos + (WRIST_TOGGLE_INCREMENT / 180.0);
        setWristPosition(newWristPos);
    }

    public void toggleTwistCW()
    {
        double newTwistPos = twistPos + (TWIST_TOGGLE_INCREMENT / 180.0);
        setTwistPos(newTwistPos);
    }

    public void toggleTwistCCW()
    {
        double newTwistPos = twistPos - (TWIST_TOGGLE_INCREMENT / 180.0);
        setTwistPos(newTwistPos);
    }

    public boolean gripperClosed()
    {
        return leftGripperClosed || rightGripperClosed;
    }

    public void toggleGrippersLeft()
    {
        if (leftGripPos != LEFT_GRIP_DROP)
        {
            dropLeft();
        }
        else
        {
            holdPixelLeft();
        }
    }

    public void toggleGripperRight()
    {
        if (rightGripPos != RIGHT_GRIP_DROP)
        {
            dropRight();
        }
        else
        {
            holdPixelRight();
        }
    }

    public boolean clearedTransferZone()
    {
        return clearOfTransferZone;
    }

    public void update()
    {
        if (wrist != null)
        {
            wristPos = wrist.getPosition();
        }
        if (leftGripper != null)
        {
            double tempPos = leftGripper.getPosition();
            leftGripperClosed = tempPos != leftGripPos && tempPos == GripperPositions.GRIPPED.leftGripPos;
            leftGripPos = tempPos;
        }
        if (rightGripper != null)
        {
            double tempPos = rightGripper.getPosition();
            rightGripperClosed = tempPos != rightGripPos && tempPos == GripperPositions.GRIPPED.rightGripPos;
            rightGripPos = tempPos;
        }
        if (twist != null)
        {
            twistPos = twist.getPosition();
        }
        if (lElbow != null)
        {
            lElbowPos = lElbow.getPosition();
            clearOfTransferZone = lElbowPos <= Positions.AUTO_INIT.lElbowPos;
        }
        if (rElbow != null)
        {
            rElbowPos = rElbow.getPosition();
        }

        if (TELEM)
        {
            telemetry.addData("wrist position", wristPos);
            telemetry.addData("leftGrip Position", leftGripPos);
            telemetry.addData("rightGrip Position", rightGripPos);
            telemetry.addData("twist position", twistPos);
            telemetry.addData("lElbow position", lElbowPos);
            telemetry.addData("rElbow Position", rElbowPos);
        }
    }

}

