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
            hardwareHandler.intakeAngle(180);

        if (gamepad2.y)
            hardwareHandler.intakeAngle(90);

        if (gamepad2.y)
            hardwareHandler.intakeAngle(0);


        hardwareHandler.toggleLift(y,1.0);
        hardwareHandler.toggleLift(x,-0.5);
        boolean a = gamepad1.dpad_down;
        boolean b = gamepad1.dpad_up;
        hardwareHandler.toggleSlide(a,1.0);
        hardwareHandler.toggleSlide(b,-1);
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
