package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="2LinearSlides", group="Test")
public class MotorTest extends OpMode {

    private DcMotor testMotor;
    private DcMotor testMotor2;
    @Override
    public void init() {
        // Initialize the motor from the hardware map
        testMotor = hardwareMap.get(DcMotor.class, "linearSlide");
        testMotor2 = hardwareMap.get(DcMotor.class, "linearSlideTwo");
        // Reset and configure the motor
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Control the motor power using the left joystick's Y-axis
        double power = -gamepad1.left_stick_y*0.5; // Reverse the direction for intuitive control
        testMotor.setPower(power);
        testMotor2.setPower(-power);

        // Provide feedback through telemetry
        telemetry.addData("Motor Power", power);
        telemetry.addData("Motor Position", testMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop the motor when the OpMode stops
        testMotor.setPower(0);
        testMotor2.setPower(0);
    }
}
