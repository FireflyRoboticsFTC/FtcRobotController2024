package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Climbing2", group="Test")
public class Climbing2stag extends OpMode {

    private DcMotor testMotor;
    private DcMotor testMotor2;
    @Override
    public void init() {
        // Initialize the motor from the hardware map
        testMotor = hardwareMap.get(DcMotor.class, "climb1");
        testMotor2 = hardwareMap.get(DcMotor.class, "climb2");
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
        double power = -gamepad1.left_stick_y;
        double powerTwo = -gamepad1.right_stick_y;
        testMotor.setPower(power);
        testMotor2.setPower(powerTwo);
    }

    @Override
    public void stop() {
        // Stop the motor when the OpMode stops
        testMotor.setPower(0);
        testMotor2.setPower(0);
    }
}
