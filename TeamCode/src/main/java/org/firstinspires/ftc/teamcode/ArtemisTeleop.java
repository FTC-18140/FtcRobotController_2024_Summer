package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.Robot.Delivery.TWIST_INIT;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Button.B;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Button.BACK;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Button.DPAD_DOWN;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Button.DPAD_UP;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Button.LEFT_BUMPER;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Button.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Button.RIGHT_STICK_BUTTON;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Button.X;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Button.Y;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Button.LEFT_STICK_BUTTON;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Trigger.LEFT_TRIGGER;
import static org.firstinspires.ftc.teamcode.Robot.TBDGamepad.Trigger.RIGHT_TRIGGER;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Delivery;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.LinearSlide;
import org.firstinspires.ftc.teamcode.Robot.TBDGamepad;
import org.firstinspires.ftc.teamcode.Robot.ThunderbotAuto2023;

@TeleOp(name = "Artemis Teleop", group = "Teleop")
public class ArtemisTeleop extends OpMode
{

    public static double WRIST_INCREMENT = 0.025;
    public static double WRIST_POSITION = Delivery.WRIST_INIT;
    public static double TWIST_INCREMENT = 0.01;
    public static double TWIST_POSITION = TWIST_INIT;
    public static double ELBOW_INCREMENT = 0.0035;
    public static double ELBOW_POSITION = Delivery.ELBOW_INIT;
    public static double INTAKE_INCREMENT = 0.01;
    public static double INTAKE_POSITION = Intake.INTAKEELBOW_INIT;
//    Thunderbot2023 robot = new Thunderbot2023();
    ThunderbotAuto2023 robot = new ThunderbotAuto2023();
    boolean toggle = false;

    TBDGamepad tbdGamepad1;
    TBDGamepad tbdGamepad2;

    @Override
    public void init()
    {
        tbdGamepad1 = new TBDGamepad(gamepad1);
        tbdGamepad2 = new TBDGamepad(gamepad2);
        telemetry.addData("Init", "Start");
        robot.init(hardwareMap, telemetry, false, false);
        telemetry.addData("Init", "Done");
    }

    /**
     * User-defined init_loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the init button is pressed and when the play button is pressed (or the
     * OpMode is stopped).
     * <p>
     * This method is optional. By default, this method takes no action.
     */
    @Override
    public void init_loop()
    {
        super.init_loop();

        // TODO: Can use this to test the vision processing to find the team prop.  DELETE WHEN HAPPY.
        String spikePos = robot.getSpikePos();
        telemetry.addData("Spike Pos = ", spikePos);
        telemetry.addData("Prop X:", robot.getPropX());
        telemetry.addData("Prop Y:", robot.getPropY());

    }

    @Override
    public void start()
    {
    }

    public void loop()
    {
        ////////////////////////////////////////
        // UPDATE robot, gamepads, and timers
        ////////////////////////////////////////

        robot.update();
        tbdGamepad1.update();
        tbdGamepad2.update();
        telemetry.addData("Timer:", getRuntime());
        boolean toggleManual = false;
        //////////////
        // DRIVING
        //////////////

        // Resets the measured angle of the robot
//        if(tbdGamepad1.getButton(LEFT_STICK_BUTTON) && tbdGamepad1.getButton(RIGHT_STICK_BUTTON)) {
//            robot.imu.resetYaw();
//            telemetry.addData("imu: ", "reset");
//        }

        if (robot.intake.driveSlowly())
        {
            robot.joystickDrive(tbdGamepad1.getLeftY() * 0.2, tbdGamepad1.getLeftX() * 0.2,
                    tbdGamepad1.getRightX() * 0.2);
        }
        else if (tbdGamepad1.getTrigger(RIGHT_TRIGGER) > 0.1)
        {
            robot.joystickDrive(tbdGamepad1.getLeftY() * 0.2, tbdGamepad1.getLeftX() * 0.2,
                    tbdGamepad1.getRightX() * 0.2);
        }
        else
        {
            robot.joystickDrive(tbdGamepad1.getLeftY(), tbdGamepad1.getLeftX(),
                    tbdGamepad1.getRightX());
        }

        //////////////////////
        // INTAKE UP & DOWN
        //////////////////////
        robot.intake.autoIntake(tbdGamepad1.getButton(B));

        ////////////////////`
        // INTAKE GRIPPER
        ////////////////////
        if (tbdGamepad1.getButtonPressed(X))
        {
            robot.intake.dropBoth();
        }
        else if (tbdGamepad1.getButtonPressed(Y))
        {
            robot.intake.holdPixelsBoth();
        }

        ////////////////////
        // MANDIBLE
        ////////////////////

        if (tbdGamepad1.getTriggerPressed(LEFT_TRIGGER))
        {
            robot.intake.leftMandibleToggle();
        }
        //////////////////////////////////////////////
        // GAMEPAD 1 ENDGAME
        //////////////////////////////////////////////

        ////////////////////
        // PULL-UP
        ////////////////////
        if (tbdGamepad1.getButton(DPAD_UP))
        {
            robot.liftArms.pullUp(1);
        }
        else if (tbdGamepad1.getButton(DPAD_DOWN))
        {
            robot.liftArms.pullUp(-1);
        }
        else
        {
            robot.liftArms.pullUp(0);
        }


        if (robot.blipDriver1())
        {
            tbdGamepad1.blipDriver();
        }


        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////// GAMEPAD 2 //////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //////////////////
        // BACK TO INIT
        //////////////////
        if (tbdGamepad2.getButton(LEFT_STICK_BUTTON))
        {
            robot.delivery.goTo(Delivery.Positions.ALIGN_FOR_TRANSFER);
            ELBOW_POSITION = 0.92;
            robot.linearSlide.goToLinear(LinearSlide.Positions.LEVEL_0);
        }
        else if (tbdGamepad2.getButton(RIGHT_STICK_BUTTON))
        {
            robot.delivery.goTo(Delivery.Positions.ALIGN_TO_BACKDROP);
            ELBOW_POSITION = 0.8;
            robot.linearSlide.goToLinear(LinearSlide.Positions.LEVEL_1);
            robot.intake.mandibleClose();
        }
        else if (tbdGamepad2.getButton(BACK) && tbdGamepad2.getButton(LEFT_BUMPER))
        {
            robot.linearSlide.goToLinear(LinearSlide.Positions.LEVEL_0);
        }
        else if (tbdGamepad2.getButton(BACK) && tbdGamepad2.getButton(RIGHT_BUMPER))
        {
            robot.linearSlide.goToLinear(LinearSlide.Positions.LEVEL_3);
        }


        //////////////
        // LINEAR SLIDES
        //////////////
        if (tbdGamepad2.getButtonPressed(Y))
        {
            robot.linearSlide.slideTogglePositionsUp();
        }

        //////////////////
        // DELIVERY ELBOW
        //////////////////
        if (robot.intake.clearedTransferZone())
        {
            if (tbdGamepad2.getButton(X))
            {
                ELBOW_POSITION += ELBOW_INCREMENT;
                ELBOW_POSITION = robot.delivery.setElbowPosition(ELBOW_POSITION);
            }
            else if (tbdGamepad2.getButton(B))
            {
                ELBOW_POSITION -= ELBOW_INCREMENT;
                ELBOW_POSITION = robot.delivery.setElbowPosition(ELBOW_POSITION);
            }
        }

        //////////////////
        // DELIVERY WRIST
        //////////////////

        if (robot.delivery.clearedTransferZone())
        {
            if (tbdGamepad2.getButtonPressed(DPAD_UP))
            {
                WRIST_POSITION += WRIST_INCREMENT;
                WRIST_POSITION = robot.delivery.setWristPosition(WRIST_POSITION);
                robot.delivery.toggleUp();
            }
            else if (tbdGamepad2.getButtonPressed(DPAD_DOWN))
            {
                WRIST_POSITION -= WRIST_INCREMENT;
                WRIST_POSITION = robot.delivery.setWristPosition(WRIST_POSITION);
                robot.delivery.toggleDown();
            }
        }
        else
        {
            robot.delivery.setWristPosition(Delivery.WRIST_INIT);
        }

        //////////////////
        // TWIST
        //////////////////

        ////////////////////
        // DELIVERY GRIPPER
        ////////////////////

        if (tbdGamepad2.getTriggerPressed(LEFT_TRIGGER))
        {
            robot.delivery.toggleGrippersLeft();
        }
        else if (tbdGamepad2.getTriggerPressed(RIGHT_TRIGGER))
        {
            robot.delivery.toggleGripperRight();
        }

        if (robot.blipDriver2())
        {
            tbdGamepad2.blipDriver();
        }
    }
}
