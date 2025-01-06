package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous Turning with IMUs", group = "Linear Opmode")
public class TurnAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        HardwareHandler hardware = new HardwareHandler(hardwareMap, telemetry);

        waitForStart();

        if (opModeIsActive()) {
            hardware.intakeAngle(0);
            hardware.rotateToAngle(-90, -0.5);
            //sleep(3000);
            hardware.rotateToAngle(0,0.5);
            /*hardware.holdLift(true,1450, 0.5);
            hardware.holdLift(false,0, -0.3);
            sleep(750);
            hardware.holdLift(false,0, 0);
            /*double position = hardware.getAverageEncoderPosition();
            double[] imuAngles = hardware.getIMUAngles();
            double currentYaw = imuAngles[0];
            telemetry.addData("Encoder Position and current yaw","current position " + position + "\ncurrent angle " + currentYaw );
            telemetry.update();*/
            sleep(1000);
        }
    }
}
