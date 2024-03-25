package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RiggingWinch
{

    Telemetry telemetry;

    public Servo left = null;
    public Servo right = null;
    DcMotor leftWinch = null;
    DcMotor rightWinch = null;

    // Initialize
    public void init(HardwareMap hwMap, Telemetry telem)
    {
        telemetry = telem;
        try {
            leftWinch = hwMap.dcMotor.get("lWinch");
            leftWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftWinch.setDirection(DcMotorSimple.Direction.FORWARD);
            leftWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch(Exception e) {
            telemetry.addData("lWinch not found", 0);
        }
        try {
            rightWinch = hwMap.dcMotor.get("rWinch");
            rightWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightWinch.setDirection(DcMotorSimple.Direction.REVERSE);
            rightWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch(Exception e) {
            telemetry.addData("rWinch not found", 0);
        }
    }

    public void leftWinchPower(double power) {
        if ( leftWinch != null ) { leftWinch.setPower(power); }
        else { telemetry.addData("winch left motor not initialized.", 0); }
    }

    public void rightWinchPower(double power) {
        if ( rightWinch != null ) { rightWinch.setPower(power); }
        else { telemetry.addData("winch right motor not initialized.", 0); }
    }

    public void winchPower( double power )
    {
        leftWinchPower( power );
        rightWinchPower( power );
    }

    public void update() {   }

}
