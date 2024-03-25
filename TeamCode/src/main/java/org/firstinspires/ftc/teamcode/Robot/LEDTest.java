package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class LEDTest extends OpMode {
    BlinkinLED blink;
    @Override
    public void init() {
        blink  = new BlinkinLED();
        blink.init(hardwareMap, telemetry);
    }

    @Override
    public void start(){
        blink.startTimers();
    }
    @Override
    public void loop() {
        blink.checkDeadlines();
    }
}
