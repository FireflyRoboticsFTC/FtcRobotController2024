package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous with IMUs Far", group = "Linear Opmode")
public class AutoNoIMUFar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareHandler hardware = new HardwareHandler(hardwareMap, telemetry);

        waitForStart();

        if (opModeIsActive()) {
            //hardware.intakeAngle(0);
            telemetry.addData("Step", "Moving forward to Parking Close");
            telemetry.update();
            //hardware.strafeFourWheel(0.3,true);
            sleep(800);
            hardware.strafeFourWheel(0,true);
            hardware.moveFourWheel(0.35);
            sleep(800);
            //hardware.strafeFourWheel(0.3,false);
            hardware.moveFourWheel(0);
            sleep(800);
            hardware.strafeFourWheel(0,true);
            hardware.holdLift(true,0.5);
            sleep(500);
            hardware.holdLift(false,0);
            hardware.intakeSystem(-0.5);
            sleep(750);
            hardware.holdLift(true,-0.5);
            hardware.intakeAngle(0);
            hardware.intakeSystem(0);
            sleep(500);
            hardware.holdLift(false,0);
            sleep(1000);
            //hardware.strafeFourWheel(0.3,true);
            sleep(800);
            //hardware.strafeFourWheel(0,true);
            hardware.moveFourWheel(-0.25);
            sleep(1000);
            hardware.moveFourWheel(0);

        }
    }
}

