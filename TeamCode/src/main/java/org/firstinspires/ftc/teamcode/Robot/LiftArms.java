package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftArms
{

    Telemetry telemetry;

    DcMotor leftLift = null;
    DcMotor rightLift = null;
    private double leftPower = 0;
    private double rightPower = 0;

    public void init(HardwareMap hwMap, Telemetry telem)
    {
        telemetry = telem;

        try
        {
            leftLift = hwMap.dcMotor.get("oX");
            leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception e)
        {
            telemetry.addData("leftLift not found", 0);
        }
        try
        {
            rightLift = hwMap.dcMotor.get("oY");
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        catch (Exception e)
        {
            telemetry.addData("rightLift not found", 0);
        }
    }

    public void update()
    {
        if (leftLift != null)
        {
            leftPower = leftLift.getPower();
        }
        if (rightLift != null)
        {
            rightPower = rightLift.getPower();
        }
    }

    public void pullUp(double power)
    {
        if (leftLift != null && rightLift != null)
        {
            leftLift.setPower(power * 0.9);
            rightLift.setPower(power);
        }
        else
        {
            telemetry.addData("Winch lift motor not initialized.", 0);
        }
    }
}
