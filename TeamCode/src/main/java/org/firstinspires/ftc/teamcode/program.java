package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="robotTeleOpTest", group="robotgroup")
public class program extends OpMode {
    public static class Params {
        public double angle = 0.28;
    }

    public static Params PARAMS = new Params();

    private HardwareHandler hardwareHandler;
    private boolean gpd2bPrevState = false;
    private boolean aPrevState = false;
    private boolean intakeIn = false;
    private boolean intakeOut = false;
    private boolean axPrevState = false;
    private boolean measureOut = false;
    private boolean bPrevState = false;
    private boolean measureGoing = false;
    private boolean yPrev = false;
    private boolean climbUp = false;
    private boolean xPrev = false;
    private boolean climbDown = false;
    private boolean leftPrev = false;
    private boolean climbTwoUp = false;
    private boolean rightPrev = false;
    private boolean climbTwoDown = false;
    private boolean tapeUpPrev = false;
    private boolean tapeUp = false;
    private boolean tapeDownPrev = false;
    private boolean tapeDown = false;
    private boolean slowPrev = false;
    private boolean slowOn = false;
    private double slowMode = 1;
    private boolean initial = true;
    private boolean clawPrev = false;
    private boolean claw = false;

    private ElapsedTime overallRuntime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
        runtime.reset();
    }

    @Override
    public void loop(){
        if (initial) {
            hardwareHandler.setMeasure(0);
            hardwareHandler.intakeAngle(0);
            hardwareHandler.setClaw("close");
            overallRuntime.reset();
            runtime.reset();
            initial = !(initial);
        }


        boolean y = gamepad2.dpad_up;
        boolean x = gamepad2.dpad_down;
        boolean gpd2bCurrState = gamepad2.x;
        boolean aCurrState = gamepad2.a;
        boolean axCurrState = gamepad1.dpad_down;
        boolean bCurrState = gamepad1.dpad_up;
        boolean climbTwoUpCurr = gamepad2.dpad_left;
        boolean climbTwoDownCurr = gamepad2.dpad_right;
        boolean slowCurr = gamepad1.b;
        boolean tapeUpCurr = gamepad1.dpad_right;
        boolean tapeDownCurr = gamepad1.dpad_left;
        boolean clawButton = gamepad2.y;


        if (slowCurr && !slowPrev) {
            if (!slowOn)
                slowMode = 0.6;
            else
                slowMode = 1;
            slowOn = !(slowOn);
        }

        double c = 0.7;
        double f = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x * 0.5 / 0.65;
        double s = gamepad1.left_stick_x;
        double speed = Math.max(Math.max(f * f, r * r), s * s) * c * slowMode;
        hardwareHandler.moveWithPower(f, r, s, speed);

        if (overallRuntime.milliseconds() > 90*1000 && overallRuntime.milliseconds() < 91*1000) {
            hardwareHandler.climbOn(-1);
        } if (overallRuntime.milliseconds() > 95*1000 && overallRuntime.milliseconds() < 96*1000) {
            hardwareHandler.climbOn(0);
        }

        if (aCurrState && !aPrevState) {
            if (!intakeIn)
                hardwareHandler.intakeSystem(1);
            else
                hardwareHandler.intakeSystem(0);
            intakeIn = !(intakeIn);
        }

        if (gpd2bCurrState && !gpd2bPrevState) {
            if (!intakeOut)
                hardwareHandler.intakeSystem(-1);
            else
                hardwareHandler.intakeSystem(0);
            intakeOut = !(intakeOut);
        }

        if (gamepad1.y)
            hardwareHandler.intakeAngle(0);

        if (gamepad1.x)
            hardwareHandler.intakeAngle(0.0775);

        if (gamepad1.a)
            hardwareHandler.intakeAngle(PARAMS.angle);

        if (clawButton && !clawPrev) {//open .175 right, 0.8289 left
            if (!claw)
                hardwareHandler.setClaw("open");
            else
                hardwareHandler.setClaw("close");
            claw = !claw;
        }

        if (climbUp && (runtime.milliseconds() < 6000 && runtime.milliseconds() > 4900)) {
            hardwareHandler.joystickLiftOne(-0.4);
            hardwareHandler.joystickLiftTwo(-0.4);
        } else {
            hardwareHandler.joystickLiftOne(gamepad2.left_stick_y * 0.5);
            hardwareHandler.joystickLiftTwo(gamepad2.left_stick_y * 0.5);
        }

        hardwareHandler.measureAngle(gamepad2.right_stick_y);
        hardwareHandler.measurePosition();

        /*if (gamepad2.y) {
            hardwareHandler.setMeasure(0.15);
        }*/

        if (x && !xPrev) {
            if (!climbUp) {
                hardwareHandler.climbOn(1);
                runtime.reset();
            } else
                hardwareHandler.climbOn(0);
            climbUp = !(climbUp);
        }

        if (climbUp && runtime.milliseconds() > 5300) {
            hardwareHandler.climbOn(0);
            climbUp = !climbUp;
        }

        if (y && !yPrev) {
            if (!climbDown) {
                hardwareHandler.climbOn(-1);
                runtime.reset();
            } else
                hardwareHandler.climbOn(0);
            climbDown = !(climbDown);
        }

        telemetry.addData("runtime", runtime);
        telemetry.addData("boolean", climbDown);

        if (climbDown && runtime.milliseconds() > 5000) {
            hardwareHandler.climbOn(0);
            climbDown = !(climbDown);
        }

        /*if(climbTwoUpCurr && !rightPrev) {
            if (!climbTwoUp)
                hardwareHandler.climbTw(-1);
            else
                hardwareHandler.climbTw(0);
            climbTwoUp = !(climbTwoUp);
        }

        if(climbTwoDownCurr && !leftPrev) {
            if (!climbTwoDown)
                hardwareHandler.climbTw(1);
            else
                hardwareHandler.climbTw(0);
            climbTwoDown = !(climbTwoDown);
        }*/

        /*if(tapeUpCurr && !tapeUpPrev) {
            if (!tapeUp)
                hardwareHandler.launchMeasure(-1);
            else
                hardwareHandler.launchMeasure(0);
            tapeUp = !(tapeUp);
        }

        if(tapeDownCurr && !tapeDownPrev) {
            if (!tapeDown)
                hardwareHandler.launchMeasure(1);
            else
                hardwareHandler.launchMeasure(0);
            tapeDown = !(tapeDown);
        }*/

        /*if(bCurrState && !bPrevState) {
            if (!measureGoing) {
                hardwareHandler.launchMeasure(-1);
                hardwareHandler.climbTw(-0.7);
            } else {
                hardwareHandler.launchMeasure(0);
                hardwareHandler.climbTw(0);
            }
            measureGoing = !(measureGoing);
        }



        if(axCurrState && !axPrevState) {
            if (!measureOut) {
                hardwareHandler.launchMeasure(1);
                hardwareHandler.climbTw(0.7);
            } else {
                hardwareHandler.launchMeasure(0);
                hardwareHandler.climbTw(0);
            }
            measureOut = !(measureOut);
        }*/


        //hardwareHandler.climbOn(-gamepad2.left_stick_y); //stage 1
        //hardwareHandler.climbTw(gamepad2.right_stick_y); //stage 2

        //hardwareHandler.holdLift(gamepad1.x,1); //down
        //hardwareHandler.holdLift(gamepad1.y,-1); //up
        //hardwareHandler.toggleLift(x,1);
        //hardwareHandler.toggleLift(y,-0.5);
        /*boolean a = gamepad1.dpad_down;
        boolean b = gamepad1.dpad_up;
        boolean z = gamepad2.dpad_down;
        boolean v = gamepad2.dpad_up;
        hardwareHandler.toggleSlide(a,1.0);
        hardwareHandler.toggleSlide(b,-1);
        hardwareHandler.toggleSlideTwo(z,1.0);
        hardwareHandler.toggleSlideTwo(v,-1);*/

        // Declare the climbTw variable and the previous button state
        /*boolean climbTw = false; // Initially off
        boolean prevBumperState = false; // Tracks the previous state of the bumper

            // Check if the bumper button is pressed and toggle climbTw
            if (gamepad1.left_bumper && !prevBumperState) {
                climbTw = !climbTw; // Toggle the state of climbTw
            }

            // Update the previous bumper state
            prevBumperState = gamepad1.left_bumper;

            // Now you can use climbTw elsewhere in your code
            if (climbTw) {
                hardwareHandler.climbTw(1);
            } else {
                hardwareHandler.climbTw(0);
            }

        // Declare the climbTw variable and the previous button state
        boolean climbTwTw = false; // Initially off
        boolean prevBumperState2 = false; // Tracks the previous state of the bumper

            // Check if the bumper button is pressed and toggle climbTw
            if (gamepad1.right_bumper && !prevBumperState2) {
                climbTw = !climbTw; // Toggle the state of climbTw
            }

            // Update the previous bumper state
            prevBumperState2 = gamepad1.right_bumper;

            // Now you can use climbTw elsewhere in your code
            if (climbTwTw) {
                hardwareHandler.climbTw(-1);
            } else {
                hardwareHandler.climbTw(0);
            }*/

        /*if(axCurrState && !axPrevState) {
            if (!measureOut)
                hardwareHandler.measureAngle(60) ;
            else
                hardwareHandler.measureAngle(0);
            measureOut = !(measureOut);
        }*/

        gpd2bPrevState = gpd2bCurrState;
        aPrevState = aCurrState;
        axPrevState = axCurrState;
        bPrevState = bCurrState;
        yPrev = y;
        xPrev = x;
        leftPrev = climbTwoDownCurr;
        rightPrev = climbTwoUpCurr;
        slowPrev = slowCurr;
        tapeUpPrev = tapeUpCurr;
        tapeDownPrev = tapeDownCurr;
        clawPrev = clawButton;
    }
}
