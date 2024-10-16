package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="robotTeleOpTest", group="robotgroup")
public class program extends OpMode {
    private HardwareHandler hardwareHandler;
    private boolean yPrevState = false;
    private boolean slideDown = false;

    Gamepad gamePad1;

    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        boolean y = gamepad1.y;
        boolean x = gamepad1.x;
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






    }
}
