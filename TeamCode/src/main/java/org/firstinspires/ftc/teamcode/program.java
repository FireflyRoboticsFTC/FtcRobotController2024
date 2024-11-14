package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="robotTeleOpTest", group="robotgroup")
public class program extends OpMode {
    private HardwareHandler hardwareHandler;
    private boolean gpd2bPrevState = false;
    private boolean aPrevState = false;
    private boolean intakeIn = false;
    private boolean intakeOut = false;
    private boolean axPrevState = false;
    private boolean measureOut = false;
    private boolean bPrevState = false;
    private boolean measureGoing = false;
    Gamepad gamePad1;

    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        boolean y = gamepad1.y;
        boolean x = gamepad1.x;
        boolean gpd2bCurrState = gamepad2.left_bumper;
        boolean aCurrState = gamepad2.right_bumper;
        boolean axCurrState = gamepad1.a;
        boolean bCurrState = gamepad1.b;

        if (gamepad2.y)
            hardwareHandler.intakeAngle(0);

        if (gamepad2.b)
            hardwareHandler.intakeAngle(0.2);

        if (gamepad2.a)
            hardwareHandler.intakeAngle(0.425);



        hardwareHandler.climbOn(-gamepad2.left_stick_y); //stage 1
        hardwareHandler.climbTw(gamepad2.right_stick_y); //stage 2
        hardwareHandler.toggleLift(x,1);
        hardwareHandler.toggleLift(y,-0.5);
        /*boolean a = gamepad1.dpad_down;
        boolean b = gamepad1.dpad_up;
        boolean z = gamepad2.dpad_down;
        boolean v = gamepad2.dpad_up;
        hardwareHandler.toggleSlide(a,1.0);
        hardwareHandler.toggleSlide(b,-1);
        hardwareHandler.toggleSlideTwo(z,1.0);
        hardwareHandler.toggleSlideTwo(v,-1);*/

        // Declare the climbTw variable and the previous button state
        boolean climbTw = false; // Initially off
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
            }







        double c = 0.65;

        double f = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x * 0.5 / 0.65;
        double s = gamepad1.left_stick_x;
        double speed = Math.max(Math.max(f * f, r * r), s * s) * c;
        hardwareHandler.moveWithPower(f, r, s, speed);

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

        if(axCurrState && !axPrevState) {
            if (!measureOut)
                hardwareHandler.measureAngle(60) ;
            else
                hardwareHandler.measureAngle(0);
            measureOut = !(measureOut);
        }

        if(bCurrState && !bPrevState) {
            if (!measureGoing)
                hardwareHandler.launchMeasure(1);
            else
                hardwareHandler.launchMeasure(0);
            measureGoing = !(measureGoing);
        }


        gpd2bPrevState = gpd2bCurrState;
        aPrevState = aCurrState;
        axPrevState = axCurrState;
        bPrevState = bCurrState;





    }
}
