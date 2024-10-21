package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Basic Single Motor Teleop", group="Test")
public class MotorTest extends OpMode {

    private DcMotor testMotor;

    @Override
    public void init() {
        // Initialize the motor from the hardware map
        testMotor = hardwareMap.get(DcMotor.class, "linearSlide");

        // Reset and configure the motor
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Control the motor power using the left joystick's Y-axis
        double power = -gamepad1.left_stick_y; // Reverse the direction for intuitive control
        testMotor.setPower(power);

        // Provide feedback through telemetry
        telemetry.addData("Motor Power", power);
        telemetry.addData("Motor Position", testMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop the motor when the OpMode stops
        testMotor.setPower(0);
    }
}
