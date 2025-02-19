package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Servo Test", group="Test")
public class ServoTest extends LinearOpMode {

    //private Servo testServo;
    private Servo leftLiftAngle;
    private Servo rightLiftAngle;

    @Override
    public void runOpMode() {
        // Initialize the servo from your hardware map
        leftLiftAngle = hardwareMap.servo.get("leftLiftAngle");
        rightLiftAngle = hardwareMap.servo.get("rightLiftAngle");
        //testServo = hardwareMap.get(Servo.class, "test_servo");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            // Use gamepad buttons to control the servo
            if (gamepad1.a) {
                leftLiftAngle.setPosition(0);
                //rightLiftAngle.setPosition(1);
                //testServo.setPosition(0.0);  // Move to 0 position
            } else if (gamepad1.b) {
                leftLiftAngle.setPosition(1);
                //rightLiftAngle.setPosition(1-0.31);
                //testServo.setPosition(1.0);
            } else if (gamepad1.x) {
                rightLiftAngle.setPosition(0);
            } else if (gamepad1.y) {
                rightLiftAngle.setPosition(1);
            }

            double leftCurrPos = leftLiftAngle.getPosition();
            double rightCurrPos = rightLiftAngle.getPosition();
            leftLiftAngle.setPosition(leftCurrPos + gamepad1.left_stick_y * 0.001);
            rightLiftAngle.setPosition(rightCurrPos + gamepad1.right_stick_y * 0.001);

            // Send telemetry data back to the Driver Station for monitoring
            //telemetry.addData("Servo Position", testServo.getPosition());
            telemetry.addData("Left Servo Position", leftLiftAngle.getPosition());
            telemetry.addData("Right Servo Position", rightLiftAngle.getPosition());
            telemetry.update();
        }
    }
}
