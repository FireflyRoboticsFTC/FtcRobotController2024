package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "IMU Calibration", group = "Calibration")
public class IMUTest extends LinearOpMode {

    private HardwareHandler hardwareHandler;

    @Override
    public void runOpMode() {
        // Initialize the HardwareHandler
        hardwareHandler = new HardwareHandler(hardwareMap);

        // Wait for the start button to be pressed
        telemetry.addData("Status", "Initializing IMU...");
        telemetry.update();

        // Ensure the IMU is initialized and calibrated
        while (!isStopRequested()) {

            double[] list = hardwareHandler.getIMUAngles();


            telemetry.addData("IMU Yaw", list[0]);
            telemetry.addData("IMU Pitch", list[1]);
            telemetry.addData("IMU Roll", list[2]);
            telemetry.update();
        }

            // Wait for the start button to be pressed
        waitForStart();

            // Once started, you can add further code or just loop to keep showing calibration status
        while (opModeIsActive()) {
                telemetry.addData("IMU Status", "Calibrated!");
                telemetry.update();
                sleep(100); // Update every 100ms
        }

    }

}

