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
            hardware.intakeAngle(0);
            telemetry.addData("Step", "Moving forward to Parking Close");
            telemetry.update();
            hardware.holdLift(true,1780, 0.5);
            hardware.moveForwardWithHeading(0.6,13.5,0);
            sleep(500);
            //release counterweight for slides
            //hardware.holdLift(false,0);
            hardware.moveBackwardWithHeading(-0.2,-3,0);
            hardware.moveBackwardWithHeading(-0.6,-9.5,0);
            hardware.holdLift(false,0, -0.3);
            sleep(750);
            hardware.holdLift(false,0, 0);

            hardware.rotateToAngle(-90,-0.5);
            hardware.intakeSystem(1);
            hardware.moveForwardWithHeading(0.6,16.5,0);
            hardware.intakeAngle(.4);
            sleep(2000);
            hardware.intakeSystem(0);
            hardware.intakeAngle(0);
            hardware.moveBackwardWithHeading(-0.6,-20.5,0);
            hardware.rotateToAngle(0,0.5);

            hardware.holdLift(true,1780, 0.5);
            hardware.moveForwardWithHeading(0.6,13,0);
            sleep(500);
            //release counterweight for slides
            //hardware.holdLift(false,0);
            hardware.moveBackwardWithHeading(-0.2,-3,0);
            hardware.moveBackwardWithHeading(-0.6,-10,0);
            hardware.holdLift(false,0, -0.3);
            sleep(750);
            hardware.holdLift(false,0, 0);



        }
    }
}

