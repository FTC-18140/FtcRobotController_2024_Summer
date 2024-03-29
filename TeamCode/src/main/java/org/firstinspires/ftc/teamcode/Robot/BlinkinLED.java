package org.firstinspires.ftc.teamcode.Robot;
/*
 * Copyright (c) 2018 Craig MacFarlane
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Craig MacFarlane nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/**
 * Connects to the Blinkin LED Driver and is used to establish time-based LED displays for the robot to
 * help the drivers know when end game is about to occur and when the match is about to be complete.
 */
public class BlinkinLED
{

    public final static int TELEOP_TIME = 75; // 45 seconds remaining -- 75 seconds elapsed
    public final static int ENDGAME_TIME = TELEOP_TIME + 15;  // 30 seconds remaining -- 90 seconds elapsed
    public final static int FINAL_TIME = ENDGAME_TIME + 15; // 15 seconds remaining --  105 seconds elapsed
    public final static int GAMEOVER_TIME = 120; // 0 seconds remaining -- 120 seconds elapsed

    public final static RevBlinkinLedDriver.BlinkinPattern TELEOP_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLACK; // GRAY
    public final static RevBlinkinLedDriver.BlinkinPattern READY_4_ENDGAME_PATTERN = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    public final static RevBlinkinLedDriver.BlinkinPattern ENDGAME_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
    public final static RevBlinkinLedDriver.BlinkinPattern LAST_15_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED;
    public final static RevBlinkinLedDriver.BlinkinPattern GAMEOVER_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLACK;

    boolean displayDeadlines = false;

    Telemetry telemetry;
    RevBlinkinLedDriver blinkinLedDriver = null;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Deadline teleopLimit;
    Deadline endgameLimit;
    Deadline lastFifteen;
    Deadline gameOver;

    /**
     * Method which allows deadline timer values to be displayed in telemetry
     */
    public void showTimers()
    {
        displayDeadlines = true;
    }

    /**
     * Initialize the Blinkin LED driver with the hardware map and setup the patters to display.
     *
     * @param hMap
     * @param telem
     */
    public void init(HardwareMap hMap, Telemetry telem)
    {
        telemetry = telem;
        try
        {
            blinkinLedDriver = hMap.get(RevBlinkinLedDriver.class, "blinkin");
            pattern = TELEOP_PATTERN;
            blinkinLedDriver.setPattern(pattern);
        }
        catch (Exception e)
        {
            telemetry.addData("led not found in config file", 0);
        }

        teleopLimit = new Deadline(TELEOP_TIME, TimeUnit.SECONDS);
        endgameLimit = new Deadline(ENDGAME_TIME, TimeUnit.SECONDS);
        lastFifteen = new Deadline(FINAL_TIME, TimeUnit.SECONDS);
        gameOver = new Deadline(GAMEOVER_TIME, TimeUnit.SECONDS);
    }

    /**
     * Batch reset of all timers so that they can begin timing in unison.
     */
    public void startTimers()
    {
        teleopLimit.reset();
        endgameLimit.reset();
        lastFifteen.reset();
        gameOver.reset();
    }

    /**
     * Internal method which changes the LED pattern based on the state of the individual timers.
     */
    public void checkDeadlines()
    {
        if (displayDeadlines)
        {
            telemetry.addData("TeleopLimit:", teleopLimit.timeRemaining(TimeUnit.SECONDS));
            telemetry.addData("EndgameLimit:", endgameLimit.timeRemaining(TimeUnit.SECONDS));
            telemetry.addData("LastFive:", lastFifteen.timeRemaining(TimeUnit.SECONDS));
        }
        if (gameOver.hasExpired()) // 120 seconds
        {
            pattern = GAMEOVER_PATTERN;
            setPattern();
        }
        else if (lastFifteen.hasExpired()) // 105 seconds
        {
            pattern = LAST_15_PATTERN;
            setPattern();
        }
        else if (endgameLimit.hasExpired())  // 90 seconds
        {
            pattern = ENDGAME_PATTERN;
            setPattern();
        }
        else if (teleopLimit.hasExpired())  // 75 seconds
        {
            pattern = READY_4_ENDGAME_PATTERN;
            setPattern();
        }
    }

    /**
     * Internal method which communicates with the Blinkin hardware and tells it the current pattern
     * to display
     */
    protected void setPattern()
    {
        if (blinkinLedDriver != null)
        {
            blinkinLedDriver.setPattern(pattern);
        }
    }
}
