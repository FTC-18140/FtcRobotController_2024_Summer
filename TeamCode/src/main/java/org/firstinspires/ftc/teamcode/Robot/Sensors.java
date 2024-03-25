package org.firstinspires.ftc.teamcode.Robot;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Sensors {
    Telemetry  telemetry;
    ColorSensor lDistance = null;
    ColorSensor rDistance = null;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        try {
            lDistance = hwMap.get(ColorSensor.class, "lDistance");
        } catch (Exception e) {
            telemetry.addData("Left Distance Sensor not found", 0);
        }
        try {
            rDistance = hwMap.get(ColorSensor.class, "rDistance");
        } catch (Exception e) {
            telemetry.addData("Right Distance Sensor not found", 0);
        };
    }

    public void leftDistanceDetect() {
        double leftDistance = ((DistanceSensor) lDistance).getDistance(DistanceUnit.CM);

        telemetry.addData("Left Distance = ", leftDistance);
    }
    public void rightDistanceDetect() {
        double rightDistance = ((DistanceSensor) rDistance).getDistance(DistanceUnit.CM);

        telemetry.addData("Right Distance =  ", rightDistance);
    }

}
