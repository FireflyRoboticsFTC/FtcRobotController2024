package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Servo Test", group="Test")
public class ServoTest extends LinearOpMode {

    private Servo testServo;

    @Override
    public void runOpMode() {
        // Initialize the servo from your hardware map
        testServo = hardwareMap.get(Servo.class, "test_servo");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Use gamepad buttons to control the servo
            if (gamepad1.a) {
                testServo.setPosition(0.0);  // Move to 0 position
            } else if (gamepad1.b) {
                testServo.setPosition(1.0);  // Move to max position
            }

            // Send telemetry data back to the Driver Station for monitoring
            telemetry.addData("Servo Position", testServo.getPosition());
            telemetry.update();
        }
    }
}
