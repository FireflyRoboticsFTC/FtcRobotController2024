package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="robotTeleOpTest", group="robotgroup")
public class program extends OpMode {

    private HardwareHandler hardwareHandler;

    private double slowModifier = 1;

    private boolean intakeIn = false;
    private boolean intakeOut = false;
    private boolean measureOut = false;
    private boolean measureGoing = false;
    private boolean climbUp = false;
    private boolean climbDown = false;
    private boolean climbTwoUp = false;
    private boolean climbTwoDown = false;
    private boolean tapeUp = false;
    private boolean tapeDown = false;
    private boolean slowOn = false;
    private boolean initial = true;
    private boolean clawToggle = false;
    private boolean autoClimb = false;
    private boolean liftUp = false;
    private boolean rumbleIntake= false;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime climbRuntime = new ElapsedTime();
    private final ElapsedTime liftRuntime = new ElapsedTime();

    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (initial) {
            hardwareHandler.setMeasure(0.05);
            hardwareHandler.intakeAngle(0);
            hardwareHandler.setClaw("close");
            climbRuntime.reset();
            runtime.reset();
            initial = !(initial);
        }

        telemetry.addData("runtime", runtime);
        hardwareHandler.slideTelemetry();

        boolean slowMode = gamepad1.b;
        boolean holdSlowMode = gamepad1.left_trigger != 0;
        boolean highArm = gamepad1.y || gamepad2.y;
        boolean midArm = gamepad1.x || gamepad2.x || gamepad2.dpad_down;
        boolean lowArm = gamepad1.a || gamepad2.a;
        boolean claw = gamepad1.right_bumper;
        boolean intakeSpinIn = gamepad1.left_bumper || gamepad2.dpad_left || gamepad2.dpad_down;
        boolean intakeSpinOut = gamepad2.dpad_right;
        boolean clipLift = gamepad2.dpad_up;
        boolean climbOneUp = gamepad1.dpad_up;
        boolean climbOneDown = gamepad1.dpad_down;

        double c = 1;
        double f = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x /* 0.5 / 0.65*/;
        double s = gamepad1.left_stick_x;
        double speed = Math.max(Math.max(f * f, r * r), s * s) * c * slowModifier;
        hardwareHandler.moveWithPower(f, r, s, speed);

        if (slowMode && !previousGamepad1.b)
            slowOn = !(slowOn);
        if (holdSlowMode)
            slowModifier = 0.6;
        else if (slowOn)
            slowModifier = 0.4;
        else
            slowModifier = 1;

        if (!holdSlowMode && previousGamepad1.left_trigger != 0 && slowOn)
            slowOn = false;

        if (highArm)
            hardwareHandler.intakeAngle(0);
        if (midArm)
            hardwareHandler.intakeAngle(0.185);
        if (lowArm)
            hardwareHandler.intakeAngle(0.38);

        if (intakeSpinIn && !(previousGamepad1.left_bumper || previousGamepad2.dpad_left || previousGamepad2.dpad_down)) {
            intakeIn = !(intakeIn);
            intakeOut = false;
            if (intakeIn)
                hardwareHandler.intakeSystem(1);
            else {
                hardwareHandler.intakeSystem(0);
            }
        }

        if (intakeSpinOut && !previousGamepad2.dpad_right) {
            intakeOut = !(intakeOut);
            intakeIn = false;
            if (intakeOut)
                hardwareHandler.intakeSystem(-1);
            else {
                hardwareHandler.intakeSystem(0);
            }
        }

        if (claw && !previousGamepad1.right_bumper) {//open .175 right, 0.8289 left
            clawToggle = !clawToggle;
            if (clawToggle)
                hardwareHandler.setClaw("open");
            else
                hardwareHandler.setClaw("close");
        }

        if (clipLift && !previousGamepad2.dpad_up) { //assumes slides are starting at bottom
            liftUp = !(liftUp);
            if (liftUp) {
                hardwareHandler.setLiftPos(1200);
                liftRuntime.reset();
            } else
                hardwareHandler.setLiftPos(-1200);
        }

        if (liftUp) {
            if (liftRuntime.milliseconds() > 250) {
                hardwareHandler.intakeSystem(0);
                intakeIn = false;
            }
            if (liftRuntime.milliseconds() > 500) {
                hardwareHandler.intakeAngle(0);
                liftUp = false;
            }
        }

        if (gamepad2.left_stick_y != 0) //if you move the joystick at all no more automatic
            hardwareHandler.joystickLift(-gamepad2.left_stick_y);
        else
            hardwareHandler.updateLift();

        if (runtime.seconds() > 60 && !rumbleIntake) {
            gamepad1.rumbleBlips(3);
            rumbleIntake = true;
        }

        if (climbOneUp && !previousGamepad1.dpad_up) {
            climbUp = !(climbUp);
            climbDown = false;
            if (climbUp) {
                hardwareHandler.climbOn(1);
                climbRuntime.reset();
            } else
                hardwareHandler.climbOn(0);
        }

        if (runtime.seconds() > 90 && !autoClimb) {
            hardwareHandler.climbOn(1);
            gamepad2.rumble(500);
            climbRuntime.reset();
            climbUp = true;
            autoClimb = true;
        }

        if (climbUp && climbRuntime.milliseconds() > 5000) {
            hardwareHandler.climbOn(0);
            climbUp = false;
        }

        if (climbOneDown && !previousGamepad1.dpad_down) {
            climbDown = !(climbDown);
            climbUp = false;
            if (climbDown) {
                hardwareHandler.climbOn(-1);
                hardwareHandler.intakeAngle(0);
                hardwareHandler.setLiftPos(400);
                climbRuntime.reset();
            } else
                hardwareHandler.climbOn(0);
        }

        if (climbDown && climbRuntime.milliseconds() > 5300) {
            hardwareHandler.climbOn(0);
            climbDown = false;
        }

        hardwareHandler.measureAngle(gamepad2.right_stick_y);
        hardwareHandler.measurePosition();

        /*if (gamepad2.y) {
            hardwareHandler.setMeasure(0.15);
        }*/

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

        /*if(axCurrState && !axPrevState) {
            if (!measureOut)
                hardwareHandler.measureAngle(60) ;
            else
                hardwareHandler.measureAngle(0);
            measureOut = !(measureOut);
        }*/

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
