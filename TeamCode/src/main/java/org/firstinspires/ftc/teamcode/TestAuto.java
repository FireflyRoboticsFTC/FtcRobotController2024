package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous with IMUs", group = "Linear Opmode")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareHandler hardware = new HardwareHandler(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Step", "Moving forward with heading 0 degrees");
            telemetry.update();
            hardware.moveForwardWithHeading(0.25, 1000, 0);
            double position = hardware.getAverageEncoderPosition();
            double[] imuAngles = hardware.getIMUAngles();
            double currentYaw = imuAngles[0];
            telemetry.addData("Encoder Position and current yaw","current position " + position + "current angle " + currentYaw );
            telemetry.update();
            sleep(1000);
            telemetry.addData("Step", "Rotating to 90 degrees");
            telemetry.update();
            hardware.rotateToAngle(360,0.5);
            sleep(1000);
            telemetry.addData("Step", "Moving forward with heading 90 degrees");
            telemetry.update();
            hardware.moveForwardWithHeading(0.5, 5000, 90);
            sleep(1000);
        }
    }
}

