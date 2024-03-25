package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;

@Config
public class TBDGamepad
{
    public Gamepad gamepad;

    public static double expoYValue = 2.5;
    public static double expoXValue = 2.5;
    public boolean[] buttons = new boolean[14];
    public boolean[] oldButtons = new boolean[14];
    public boolean[] changed = new boolean[14];

    public boolean leftOldTrigger = false;
    public boolean leftNewTrigger = false;
    public double triggerThreshold = 0.5;
    public boolean rightOldTrigger = false;
    public boolean rightNewTrigger = false;
    public boolean leftTriggerChanged = false;
    public boolean rightTriggerChanged = false;

    public enum Button {
        A(0), B(1), X(2), Y(3), LEFT_BUMPER(4), RIGHT_BUMPER(5), BACK(6),
        START(7), DPAD_UP(8), DPAD_DOWN(9), DPAD_LEFT(10), DPAD_RIGHT(11),
        LEFT_STICK_BUTTON(12), RIGHT_STICK_BUTTON(13);
        final int index;
        Button( int ind )
        {
            this.index = ind;
        }
    }

    public enum Trigger {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

    public enum Stick {
        LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y
    }

    public TBDGamepad(Gamepad gamepad)
    {
        this.gamepad = gamepad;
        Arrays.fill(buttons, false);
        Arrays.fill(oldButtons, false);
        Arrays.fill(changed, false);
    }

    /**
     * @param button the button object
     * @return the boolean value as to whether the button is active or not
     */
    public boolean getButton(Button button) {
        boolean buttonValue = false;
        switch (button) {
            case A:
                buttonValue = gamepad.a;
                break;
            case B:
                buttonValue = gamepad.b;
                break;
            case X:
                buttonValue = gamepad.x;
                break;
            case Y:
                buttonValue = gamepad.y;
                break;
            case LEFT_BUMPER:
                buttonValue = gamepad.left_bumper;
                break;
            case RIGHT_BUMPER:
                buttonValue = gamepad.right_bumper;
                break;
            case DPAD_UP:
                buttonValue = gamepad.dpad_up;
                break;
            case DPAD_DOWN:
                buttonValue = gamepad.dpad_down;
                break;
            case DPAD_LEFT:
                buttonValue = gamepad.dpad_left;
                break;
            case DPAD_RIGHT:
                buttonValue = gamepad.dpad_right;
                break;
            case BACK:
                buttonValue = gamepad.back;
                break;
            case START:
                buttonValue = gamepad.start;
                break;
            case LEFT_STICK_BUTTON:
                buttonValue = gamepad.left_stick_button;
                break;
            case RIGHT_STICK_BUTTON:
                buttonValue = gamepad.right_stick_button;
                break;
            default:
                break;
        }
        return buttonValue;
    }

    /**
     * @param trigger the trigger object
     * @return the value returned by the trigger in question
     */
    public double getTrigger(Trigger trigger) {
        double triggerValue = 0;
        switch (trigger) {
            case LEFT_TRIGGER:
                triggerValue = gamepad.left_trigger;
                break;
            case RIGHT_TRIGGER:
                triggerValue = gamepad.right_trigger;
                break;
            default:
                break;
        }
        return triggerValue;
    }
    public boolean getTriggerBoolean(Trigger trigger) {
        boolean triggerValue = false;
        switch (trigger) {
            case LEFT_TRIGGER:
                triggerValue = gamepad.left_trigger > 0.1;
                break;
            case RIGHT_TRIGGER:
                triggerValue = gamepad.right_trigger > 0.1;
                break;
            default:
                break;
        }
        return triggerValue;
    }

    /**
     * @return the y-value on the left analog stick
     */
    public double getLeftY() {
        return -gamepad.left_stick_y;
    }


    /**
     * @return the y-value on the right analog stick
     */
    public double getRightY() {
        return -gamepad.right_stick_y;
    }

    /**
     * @return the x-value on the left analog stick
     */
    public double getLeftX() {
        return gamepad.left_stick_x;
    }

    /**
     * @return the x-value on the right analog stick
     */
    public double getRightX() {
        return gamepad.right_stick_x;
    }

    public double getExpo( Stick stick )
    {
        switch (stick)
        {
            case LEFT_X:
                return Math.pow(getLeftX(), expoXValue);
            case LEFT_Y:
                return Math.pow(getLeftY(), expoYValue);
            case RIGHT_X:
                return Math.pow(getRightX(), expoXValue);
            case RIGHT_Y:
                return Math.pow(getRightY(), expoYValue);
            default:
                return 0;
        }
    }

    public boolean getButtonPressed( Button theButton )
    {
        return changed[theButton.index] && buttons[theButton.index];
    }

    public boolean getButtonReleased( Button theButton )
    {
        return changed[theButton.index] && !buttons[theButton.index];
    }
    public boolean getTriggerPressed(Trigger theTrigger) {
            if (theTrigger == Trigger.LEFT_TRIGGER) {
                return leftTriggerChanged && leftNewTrigger;
            } else {
                return rightTriggerChanged && rightNewTrigger;
            }

    }

    public void notifyDriver(int numBlips) { gamepad.rumbleBlips(numBlips);}

    public void update() {

        System.arraycopy(buttons, 0, oldButtons, 0, 14);

        buttons[Button.A.index] = gamepad.a;
        buttons[Button.B.index] = gamepad.b;
        buttons[Button.X.index] = gamepad.x;
        buttons[Button.Y.index] = gamepad.y;
        buttons[Button.LEFT_BUMPER.index] = gamepad.left_bumper;
        buttons[Button.RIGHT_BUMPER.index] = gamepad.right_bumper;
        buttons[Button.DPAD_UP.index] = gamepad.dpad_up;
        buttons[Button.DPAD_DOWN.index] = gamepad.dpad_down;
        buttons[Button.DPAD_LEFT.index] = gamepad.dpad_left;
        buttons[Button.DPAD_RIGHT.index] = gamepad.dpad_right;
        buttons[Button.BACK.index] = gamepad.back;
        buttons[Button.START.index] = gamepad.start;
        buttons[Button.LEFT_STICK_BUTTON.index] = gamepad.left_stick_button;
        buttons[Button.RIGHT_STICK_BUTTON.index] = gamepad.right_stick_button;
        leftOldTrigger = leftNewTrigger;
        leftNewTrigger = gamepad.left_trigger > triggerThreshold;
        leftTriggerChanged = leftOldTrigger != leftNewTrigger;
        rightOldTrigger = rightNewTrigger;
        rightNewTrigger = gamepad.right_trigger > triggerThreshold;
        rightTriggerChanged = rightOldTrigger != rightNewTrigger;

        for ( int i = 0; i < 14; i++ )
        {
            changed[i] = oldButtons[i] != buttons[i];
        }
    }


}
