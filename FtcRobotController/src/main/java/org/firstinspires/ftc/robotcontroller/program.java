package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="robotHopeCore", group="robotgroup")
public class program extends OpMode {
    private HardwareHandler hardwareHandler;

    Gamepad gamePad1;

    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap);
    }

    @Override
    public void loop(){
        double c = 0.65;
        double d = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x * 0.5 / 0.65;
        double speed = Math.max(d * d,r * r) * c;

        hardwareHandler.moveWithPower(d, r, speed);
    }

}
