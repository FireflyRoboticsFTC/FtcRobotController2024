package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Encoder Test", group = "Calibration")
public class EncoderTest extends OpMode {

    private HardwareHandler hardware;

    @Override
    public void init() {
        hardware = new HardwareHandler(hardwareMap, telemetry);
        hardware.telemetryEncoderPosition();
    }

    @Override
    public void loop(){
        hardware.telemetryEncoderPosition();


        hardware.moveFourWheel(.4*gamepad1.left_stick_y);
    }
}
