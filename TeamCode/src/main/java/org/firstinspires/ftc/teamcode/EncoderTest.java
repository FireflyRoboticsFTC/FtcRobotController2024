package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Encoder Test", group = "Calibration")
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        HardwareHandler hardware = new HardwareHandler(hardwareMap, telemetry);

        while (!isStopRequested()) {
            hardware.telemetryEncoderPosition();
        }
    }
}
