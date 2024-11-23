package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous with IMU Far Nothing", group = "Linear Opmode")
public class AutoNoIMUFarNothing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareHandler hardware = new HardwareHandler(hardwareMap, telemetry);

        waitForStart();

        if (opModeIsActive()) {
            //hardware.intakeAngle(0);

            telemetry.addData("Step", "Moving forward to Parking Close");
            telemetry.update();
            hardware.moveFourWheel(-0.25);
            sleep(1400);

        }
    }
}
