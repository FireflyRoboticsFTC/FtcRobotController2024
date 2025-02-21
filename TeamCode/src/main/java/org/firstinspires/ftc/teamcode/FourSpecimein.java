package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

@Config
@Autonomous(name = "rr test auto 4", group = "Autonomous")
public class FourSpecimein extends LinearOpMode {
    public static double subY = 28;
    public static double specimenY = 55;
    public static double specimenX = -43;
    public static double slidePos = 1250;
    public static double slideDelay = 2200;
    public static double specimenDelay = 0.4;

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

        public class LiftUpInitial implements Action {
            private boolean initialized = false;
            private double leftTargetPos = slidePos;
            private double rightTargetPos = slidePos;
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftTargetPos += linearLiftLeft.getCurrentPosition();
                    rightTargetPos += linearLiftRight.getCurrentPosition();
                    initialized = true;
                    runtime.reset();
                }

                // checks lift's current position
                double leftDiff = leftTargetPos - linearLiftLeft.getCurrentPosition();
                double rightDiff = rightTargetPos - linearLiftRight.getCurrentPosition();
                packet.put("liftDiff", leftDiff);
                if (leftDiff > 200 && rightDiff > 200) {
                    linearLiftLeft.setPower(0.8);
                    linearLiftRight.setPower(0.8);
                    return true;
                } else if (leftDiff > 0 && rightDiff > 0) {
                    linearLiftLeft.setPower(0.45 * (leftDiff / 200.0) + 0.35);
                    linearLiftRight.setPower(0.45 * (leftDiff / 200.0) + 0.35);
                    return true;
                } else {
                    linearLiftLeft.setPower(0.05);
                    linearLiftRight.setPower(0.05);
                    return false;
                }
            }
        }

        public class LiftUp implements Action {
            private boolean initialized = false;
            private double leftTargetPos = slidePos+100;
            private double rightTargetPos = slidePos+100;
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearLiftLeft.setPower(0.7);
                    linearLiftRight.setPower(0.7);
                    leftTargetPos += linearLiftLeft.getCurrentPosition();
                    rightTargetPos += linearLiftRight.getCurrentPosition();
                    initialized = true;
                }

                // checks lift's current position
                double leftDiff = leftTargetPos - linearLiftLeft.getCurrentPosition();
                double rightDiff = rightTargetPos - linearLiftRight.getCurrentPosition();
                packet.put("liftDiff", leftDiff);

                //up for 300 ms
                if (runtime.milliseconds() < 100) {
                    return true;
                }
                //fake pid from 2300 ms till reached target pos
                else if (runtime.milliseconds() > slideDelay) {
                    if (leftDiff > 200 && rightDiff > 200) {
                        linearLiftLeft.setPower(0.8);
                        linearLiftRight.setPower(0.8);
                        return true;
                    } else if (leftDiff > 0 && rightDiff > 0) {
                        linearLiftLeft.setPower(0.45 * (leftDiff / 200.0) + 0.35);
                        linearLiftRight.setPower(0.45 * (leftDiff / 200.0) + 0.35);
                        return true;
                    } else {
                        linearLiftLeft.setPower(0.08);
                        linearLiftRight.setPower(0.08);
                        return false;
                    }
                }
                //hold from 300 ms to 2300 ms
                else {
                    linearLiftLeft.setPower(0.08);
                    linearLiftRight.setPower(0.08);
                    return true;
                }
            }
        }

        public class LiftDown implements Action {
            private boolean initialized = false;
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearLiftLeft.setPower(-0.8);
                    linearLiftRight.setPower(-0.8);
                    initialized = true;
                    runtime.reset();
                }
                // checks lift's current position
                double leftPos = linearLiftLeft.getCurrentPosition();
                double rightPos = linearLiftRight.getCurrentPosition();
                packet.put("liftPos", leftPos);
                if (runtime.milliseconds() < 250) {
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

        public Action liftUp() { return new LiftUp(); }

        public Action liftUpInitial() { return new LiftUpInitial(); }

        public Action liftDown() { return new LiftDown(); }
    }

    public class Intake {
        private final CRServo leftIntake;
        private final CRServo rightIntake;
        private final Servo leftLiftAngle;
        private final Servo rightLiftAngle;
        private final Servo leftTapeMeasureAim;
        private final Servo rightTapeMeasureAim;
        private final Servo leftClaw;
        private final Servo rightClaw;

        public Intake(HardwareMap hardwareMap) {
            leftIntake = hardwareMap.crservo.get("leftIntake");
            rightIntake = hardwareMap.crservo.get("rightIntake");
            leftLiftAngle = hardwareMap.servo.get("leftLiftAngle");
            rightLiftAngle = hardwareMap.servo.get("rightLiftAngle");
            leftTapeMeasureAim = hardwareMap.servo.get("leftTapeMeasureAim");
            rightTapeMeasureAim = hardwareMap.servo.get("rightTapeMeasureAim");
            leftClaw = hardwareMap.servo.get("leftClaw");
            rightClaw = hardwareMap.servo.get("rightClaw");
        }

        public void servoStart() {
            leftLiftAngle.setPosition(0.01);
            rightLiftAngle.setPosition(1);
            leftClaw.setPosition(0.99);
            rightClaw.setPosition(0.0128);
            leftTapeMeasureAim.setPosition(.9728-0.05);
            rightTapeMeasureAim.setPosition(0.05);
        }

        public class IntakeUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                leftLiftAngle.setPosition(0.01);
                rightLiftAngle.setPosition(1);
                leftClaw.setPosition(0.99);
                rightClaw.setPosition(0.0128);
                return false;
            }
        }

        public class IntakeDown implements Action {
            /*@Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
                leftLiftAngle.setPosition(0.01+0.185);
                rightLiftAngle.setPosition(1-0.185);
                leftClaw.setPosition(0.97);
                rightClaw.setPosition(0.0328);
                return false;
            }*/
            private boolean initialized = false;
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftIntake.setPower(-1);
                    rightIntake.setPower(1);
                    leftLiftAngle.setPosition(0.01+0.18);
                    rightLiftAngle.setPosition(1-0.18);
                    leftClaw.setPosition(0.9);
                    rightClaw.setPosition(0.1);
                    initialized = true;
                    runtime.reset();
                }

                if (runtime.milliseconds() < 750) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    leftClaw.setPosition(0.97);
                    rightClaw.setPosition(0.0328);
                    return false;
                }
            }
        }

        public class Claw implements Action {
            private boolean initialized = false;
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftClaw.setPosition(0.9);
                    rightClaw.setPosition(0.1);
                    initialized = true;
                    runtime.reset();
                }

                if (runtime.milliseconds() < 800) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    leftClaw.setPosition(0.99);
                    rightClaw.setPosition(0.0128);
                    return false;
                }
            }
        }

        public Action intakeDown() { return new IntakeDown(); }

        public Action intakeUp() { return new IntakeUp(); }

        public Action claw() { return new Claw(); }
    }

    public Action sleep(double dt) { return new SleepAction(dt); }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-7.5, 61.375, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        Intake intake = new Intake(hardwareMap);
        // make a Lift instance
        Lift lift = new Lift(hardwareMap);

        Action toSubmersibleOne = drive.actionBuilder(initialPose)
                .lineToY(subY+2)
                .build();

        Action pushTwoBlock = drive.actionBuilder(new Pose2d(-7.5, subY+2, Math.toRadians(-90)))
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-34, 36), Math.PI)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-46, 16, Math.PI), Math.PI)
                .strafeTo(new Vector2d(-38, 55))
                //.strafeTo(new Vector2d(-37, 34))
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-57, 16.5, Math.PI / 2), Math.PI)
                .strafeTo(new Vector2d(-57, specimenY-6))
                .build();

        Action specimenTwo = drive.actionBuilder(new Pose2d(-57, specimenY-6, Math.toRadians(90)))
                .strafeTo(new Vector2d(-57, specimenY+2))
                .waitSeconds(specimenDelay)
                .build();

        Action toSubmersibleTwo = drive.actionBuilder(new Pose2d(-56, specimenY+2, Math.toRadians(90)))
                .setTangent(-Math.PI/3)
                .splineToLinearHeading(new Pose2d(-5, subY-.5, -Math.PI/2), -Math.PI/2, null, new ProfileAccelConstraint(-21.0,65.0))
                .build();

        Action specimenThree = drive.actionBuilder(new Pose2d(-5, subY-.5, Math.toRadians(-90)))
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(specimenX, specimenY-10, Math.PI/2), 3*Math.PI/4)
                .strafeTo(new Vector2d(specimenX, specimenY), new TranslationalVelConstraint(12))
                .waitSeconds(specimenDelay)
                .build();

        Action toSubmersibleThree = drive.actionBuilder(new Pose2d(specimenX, specimenY, Math.toRadians(90)))
                .setTangent(-Math.PI/4)
                .splineToLinearHeading(new Pose2d(-2, subY-1.5, -Math.PI/2), -Math.PI/2, null, new ProfileAccelConstraint(-20.0,60.0))
                .build();

        Action specimenFour = drive.actionBuilder(new Pose2d(-2, subY-1.5, Math.toRadians(-90)))
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(specimenX, specimenY-10, Math.PI/2), 3*Math.PI/4)
                .strafeTo(new Vector2d(specimenX, specimenY-1), new TranslationalVelConstraint(9))
                .waitSeconds(specimenDelay)
                .build();

        Action toSubmersibleFour = drive.actionBuilder(new Pose2d(specimenX, specimenY-1, Math.toRadians(90)))
                .setTangent(-Math.PI/4)
                .splineToLinearHeading(new Pose2d(1, subY-2.5, -Math.PI/2), -Math.PI/2,  null, new ProfileAccelConstraint(-20.0,60.0))
                .build();

        Action toPark = drive.actionBuilder(new Pose2d(1, subY-2.5, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-40, 61.375))
                .build();

        intake.servoStart();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        //one
                        new ParallelAction(
                                toSubmersibleOne,
                                lift.liftUpInitial()
                        ),
                        lift.liftDown(),
                        //two
                        new ParallelAction(
                                lift.liftDown(),
                                pushTwoBlock
                        ),
                        new ParallelAction(
                                specimenTwo,
                                intake.intakeDown()
                        ),
                        new ParallelAction(
                                lift.liftUp(),
                                new SequentialAction(
                                        sleep(0.1),
                                        new ParallelAction(
                                                intake.intakeUp(),
                                                toSubmersibleTwo
                                        )
                                )
                        ),
                        lift.liftDown(),
                        //three
                        new ParallelAction(
                                lift.liftDown(),
                                specimenThree,
                                new SequentialAction(
                                        sleep(2.0),
                                        intake.intakeDown()
                                )
                        ),
                        new ParallelAction(
                                lift.liftUp(),
                                new SequentialAction(
                                        sleep(0.1),
                                        new ParallelAction(
                                                intake.intakeUp(),
                                                toSubmersibleThree
                                        )
                                )
                        ),
                        lift.liftDown(),
                        //four
                        new ParallelAction(
                                lift.liftDown(),
                                specimenFour,
                                new SequentialAction(
                                        sleep(2.0),
                                        intake.intakeDown()
                                )
                        ),
                        new ParallelAction(
                                lift.liftUp(),
                                new SequentialAction(
                                        sleep(0.1),
                                        new ParallelAction(
                                                intake.intakeUp(),
                                                toSubmersibleFour
                                        )
                                )
                        ),
                        lift.liftDown(),
                        //park
                        new ParallelAction(
                                toPark,
                                lift.liftDown()
                        )
                )
        );
    }
}
