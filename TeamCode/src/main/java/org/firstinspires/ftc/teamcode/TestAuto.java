package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous with IMUs", group = "Linear Opmode")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareHandler hardware = new HardwareHandler(hardwareMap, telemetry);

        waitForStart();

        if (opModeIsActive()) {
            //hardware.intakeAngle(0);
            telemetry.addData("Step", "Moving forward with heading 0 degrees");
            telemetry.update();
            hardware.moveForwardWithHeading(0.5, 1000, 0);
            double position = hardware.getAverageEncoderPosition();
            double[] imuAngles = hardware.getIMUAngles();
            double currentYaw = imuAngles[0];
            telemetry.addData("Encoder Position and current yaw","current position " + position + "current angle " + currentYaw );
            telemetry.update();
            sleep(1000);
        }
    }
}

