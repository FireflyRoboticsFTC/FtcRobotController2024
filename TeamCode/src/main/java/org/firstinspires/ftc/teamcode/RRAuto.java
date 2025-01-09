package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import kotlin.math.UMathKt;

@Config
@Autonomous(name = "rr test auto", group = "Autonomous")
public class RRAuto extends LinearOpMode {

    public class Lift {
        private final DcMotor linearLiftLeft;
        private final DcMotor linearLiftRight;

        public Lift(HardwareMap hardwareMap) {
            linearLiftLeft = hardwareMap.dcMotor.get("linearLeft");
            linearLiftRight = hardwareMap.dcMotor.get("linearRight");

            linearLiftRight.setDirection(DcMotorSimple.Direction.REVERSE);

            linearLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            linearLiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            linearLiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            linearLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linearLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearLiftLeft.setPower(0.6);
                    linearLiftRight.setPower(0.6);
                    initialized = true;
                }

                // checks lift's current position
                double leftPos = linearLiftLeft.getCurrentPosition();
                double rightPos = linearLiftRight.getCurrentPosition();
                packet.put("liftPos", leftPos);
                if (leftPos < 1450 && rightPos < 1450) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    linearLiftLeft.setPower(0);
                    linearLiftRight.setPower(0);
                    return false;
                }
            }
        }

        public class LiftUp2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearLiftLeft.setPower(0.4);
                    linearLiftRight.setPower(0.4);
                    initialized = true;
                }

                // checks lift's current position
                double leftPos = linearLiftLeft.getCurrentPosition();
                double rightPos = linearLiftRight.getCurrentPosition();
                packet.put("liftPos", leftPos);
                if (leftPos < 1600 && rightPos < 1600) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    linearLiftLeft.setPower(0);
                    linearLiftRight.setPower(0);
                    return false;
                }
            }
        }

        public class LiftDown implements Action {
            private boolean initialized = false;
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearLiftLeft.setPower(-0.4);
                    linearLiftRight.setPower(-0.4);
                    initialized = true;
                    runtime.reset();
                }
                // checks lift's current position
                double leftPos = linearLiftLeft.getCurrentPosition();
                double rightPos = linearLiftRight.getCurrentPosition();
                packet.put("liftPos", leftPos);
                if (runtime.milliseconds() < 900) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    linearLiftLeft.setPower(0);
                    linearLiftRight.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public Action liftUp2() {
            return new LiftUp2();
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }

    public class Intake {
        private final CRServo leftIntake;
        private final CRServo rightIntake;
        private final Servo leftLiftAngle;
        private final Servo rightLiftAngle;

        public Intake(HardwareMap hardwareMap) {
            leftIntake = hardwareMap.crservo.get("leftIntake");
            rightIntake = hardwareMap.crservo.get("rightIntake");
            leftLiftAngle = hardwareMap.servo.get("leftLiftAngle");
            rightLiftAngle = hardwareMap.servo.get("rightLiftAngle");
        }

        public class IntakeDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
                leftLiftAngle.setPosition(0.37);
                rightLiftAngle.setPosition(1-.37);
                return false;
            }
        }

        public class IntakeUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                leftLiftAngle.setPosition(0);
                rightLiftAngle.setPosition(1);
                return false;
            }
        }

        public Action intakeDown() {
            return new IntakeDown();
        }

        public Action intakeUp() {
            return new IntakeUp();
        }
    }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 65, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        Intake intake = new Intake(hardwareMap);
        // make a Lift instance
        Lift lift = new Lift(hardwareMap);

        Action toSubmersible = drive.actionBuilder(initialPose)
                .lineToY(38, new TranslationalVelConstraint(17.0))
                .waitSeconds(0.8)
                .lineToY(45, new TranslationalVelConstraint(17.0))
                .build();

        Action backwards = drive.actionBuilder(new Pose2d(0, 45, Math.toRadians(-90)))
                .lineToYSplineHeading(60.5, Math.toRadians(180))
                .build();

        Action toObservationZone = drive.actionBuilder(new Pose2d(0, 60.5, Math.toRadians(180)))
                .lineToX(-36.5)
                .lineToX(-39.5, new TranslationalVelConstraint(5.0))
                .build();

        Action toSubmersible2 = drive.actionBuilder(new Pose2d(-39.5, 60.5, Math.toRadians(180)))
                .turnTo(-Math.PI / 2)
                .setTangent(0.0)
                .splineToConstantHeading(new Vector2d(-3, 38), -Math.PI / 2.0)
                .waitSeconds(0.7)
                .lineToY(45, new TranslationalVelConstraint(17.0))
                .build();

        Action toPark = drive.actionBuilder(new Pose2d(-3, 45, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-57, 64))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intake.intakeUp(),
                                toSubmersible,
                                lift.liftUp()
                        ),
                        backwards,
                        new ParallelAction(
                                lift.liftDown(),
                                toObservationZone,
                                intake.intakeDown()
                        ),
                        new ParallelAction(
                                intake.intakeUp(),
                                toSubmersible2,
                                lift.liftUp2()
                        ),
                        new ParallelAction(
                                toPark,
                                lift.liftDown()
                        )
                )
        );
    }
}
