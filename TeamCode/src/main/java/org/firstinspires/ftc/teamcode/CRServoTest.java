package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="CRServo Test", group="Test")
public class CRServoTest extends LinearOpMode {

    private CRServo testServo;

    @Override
    public void runOpMode() {
        // Initialize the servo from your hardware map
        testServo = hardwareMap.get(CRServo.class, "test_servo");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Use gamepad buttons to control the servo
            if (gamepad1.a) {
                testServo.setPower(0.0);  // Stop
            } else if (gamepad1.b) {
                testServo.setPower(1.0);  // Clockwise (?)
            } else if (gamepad1.x) {
                testServo.setPower(-1.0); // Counter-clockwise (?)
            }

            // Send telemetry data back to the Driver Station for monitoring
            telemetry.addData("Servo Position", testServo.getPower());
            telemetry.update();
        }
    }
}
