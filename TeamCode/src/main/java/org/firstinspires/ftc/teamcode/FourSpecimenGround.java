package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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
@Autonomous(name = "rr auto 4 ground", group = "Autonomous")
public class FourSpecimenGround extends LinearOpMode {
    public static double subY = 29;
    public static double specimenY = 55;
    public static double specimenX = -26;
    public static double slidePos = 1200;
    public static double slideDelay = 1200;
    public static double intakeVelocity = 12;
    public static double specimenDelay = 0.1;
    public static double clipAngleOffset1 = Math.toRadians(-90); //to submersible
    public static double clipAngleOffset2 = Math.toRadians(-94); //slow move
    public static double intakeAngleOffset = Math.toRadians(180-10); //to specimen

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

        public class LiftUpMid implements Action {
            private boolean initialized = false;
            private double leftTargetPos = slidePos+100;
            private double rightTargetPos = slidePos+100;
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearLiftLeft.setPower(0.8);
                    linearLiftRight.setPower(0.8);
                    leftTargetPos += linearLiftLeft.getCurrentPosition();
                    rightTargetPos += linearLiftRight.getCurrentPosition();
                    initialized = true;
                    runtime.reset();
                }

                // checks lift's current position
                double leftDiff = leftTargetPos - linearLiftLeft.getCurrentPosition();
                double rightDiff = rightTargetPos - linearLiftRight.getCurrentPosition();
                packet.put("liftDiff", leftDiff);

                //up for 300 ms
                if (runtime.milliseconds() < 150) {
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

        public class LiftUp implements Action {
            private boolean initialized = false;
            private double leftTargetPos = slidePos+150;
            private double rightTargetPos = slidePos+150;
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearLiftLeft.setPower(0.4);
                    linearLiftRight.setPower(0.4);
                    leftTargetPos += linearLiftLeft.getCurrentPosition();
                    rightTargetPos += linearLiftRight.getCurrentPosition();
                    initialized = true;
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

        public class LiftDown implements Action {
            private boolean initialized = false;
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearLiftLeft.setPower(-1);
                    linearLiftRight.setPower(-1);
                    initialized = true;
                    runtime.reset();
                }
                // checks lift's current position
                double leftPos = linearLiftLeft.getCurrentPosition();
                double rightPos = linearLiftRight.getCurrentPosition();
                packet.put("liftPos", leftPos);
                if (runtime.milliseconds() < 190) {
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

        public Action liftUpMid() { return new LiftUpMid(); }

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

        public class IntakeMid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
                leftLiftAngle.setPosition(0.01+0.175);
                rightLiftAngle.setPosition(1-0.175);
                leftClaw.setPosition(0.9); //0.97
                rightClaw.setPosition(0.1); //0.0328
                return false;
            }
        }

        public class IntakeDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
                leftLiftAngle.setPosition(0.01+0.39);
                rightLiftAngle.setPosition(1-0.39);
                leftClaw.setPosition(0.9); //0.97
                rightClaw.setPosition(0.1); //0.0328
                return false;
            }
        }

        public class Claw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftClaw.setPosition(0.99);
                rightClaw.setPosition(0.0128);
                return false;
            }
        }

        public Action intakeMid() { return new IntakeMid(); }

        public Action intakeUp() { return new IntakeUp(); }

        public Action intakeDown() { return new IntakeDown(); }

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
                .lineToY(31)
                .waitSeconds(0.1)
                .build();

        Action slowMove = drive.actionBuilder(new Pose2d(-7.5, subY+2.5, Math.toRadians(-90)))
                .lineToY(subY+1.5)
                .build();

        Action pushTwoBlock = drive.actionBuilder(new Pose2d(-7.5, 31, Math.toRadians(-90)))
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-34, 36), Math.PI)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-47, 16, Math.PI), Math.PI) // old x -46
                .strafeTo(new Vector2d(-41, 53), new TranslationalVelConstraint(65))
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-56, 16.5, Math.PI / 2), Math.PI, new TranslationalVelConstraint(60))
                .strafeTo(new Vector2d(-56, specimenY-5), new TranslationalVelConstraint(70))
                .strafeTo(new Vector2d(-56, specimenY+3), new TranslationalVelConstraint(18))
                .waitSeconds(specimenDelay)
                .build();

        Action toSubmersibleTwo = drive.actionBuilder(new Pose2d(-56, specimenY+3, Math.toRadians(90)))
                .setTangent(-Math.PI/3) ///change if we have time
                .splineToSplineHeading(new Pose2d(-4, subY-1, clipAngleOffset1-Math.toRadians(2)), clipAngleOffset1-Math.toRadians(2), null, new ProfileAccelConstraint(-20.0,60.0))
                .build();

        Action slowMove2 = drive.actionBuilder(new Pose2d(-4, subY-1, clipAngleOffset1-Math.toRadians(2)))
                .lineToYLinearHeading(subY-2, clipAngleOffset2)
                .build();

        Action specimenThree = drive.actionBuilder(new Pose2d(-4, subY-2, clipAngleOffset2))
                .setTangent(Math.toRadians(83))
                .splineToLinearHeading(new Pose2d(specimenX+10, 59, intakeAngleOffset), intakeAngleOffset)
                .strafeTo(new Vector2d(specimenX, 61), new TranslationalVelConstraint(intakeVelocity))
                .waitSeconds(0.1)
                .build();

        Action toSubmersibleThree = drive.actionBuilder(new Pose2d(specimenX, 63, intakeAngleOffset))
                .setTangent(Math.toRadians(-20))
                .splineToLinearHeading(new Pose2d(-0.5, subY, clipAngleOffset1), clipAngleOffset1, null, new ProfileAccelConstraint(-20.0,60.0))
                .build();

        Action slowMove3 = drive.actionBuilder(new Pose2d(-0.5, subY, clipAngleOffset1))
                .lineToYLinearHeading(subY-1, clipAngleOffset2)
                .build();

        Action specimenFour = drive.actionBuilder(new Pose2d(-0.5, subY-1, clipAngleOffset2))
                .setTangent(Math.toRadians(83))
                .splineToLinearHeading(new Pose2d(specimenX+10, 59, intakeAngleOffset), intakeAngleOffset)
                .strafeTo(new Vector2d(specimenX, 61), new TranslationalVelConstraint(intakeVelocity))
                .waitSeconds(0.1)
                .build();

        Action toSubmersibleFour = drive.actionBuilder(new Pose2d(specimenX, 62, intakeAngleOffset))
                .setTangent(Math.toRadians(-20))
                .splineToLinearHeading(new Pose2d(-6, subY+0.5, clipAngleOffset1), clipAngleOffset1,  null, new ProfileAccelConstraint(-20.0,60.0))
                .build();

        Action slowMove4 = drive.actionBuilder(new Pose2d(-6, subY+0.5, clipAngleOffset1))
                .lineToYLinearHeading(subY-0.5, clipAngleOffset2)
                .build();

        Action toPark = drive.actionBuilder(new Pose2d(-6, subY-0.5, clipAngleOffset2))
                .setTangent(Math.toRadians(90))
                //.splineToConstantHeading(new Vector2d(-40, 61.375), intakeAngleOffset)
                .splineToSplineHeading(new Pose2d(-38, 56, Math.toRadians(125)), Math.toRadians(125), new TranslationalVelConstraint(75), new ProfileAccelConstraint(-40.0,75.0))
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
                        /*new ParallelAction(
                                slowMove,
                                lift.liftDown()
                        ),*/
                        //two
                        new ParallelAction(
                                lift.liftDown(),
                                pushTwoBlock,
                                new SequentialAction(
                                        sleep(7.0),
                                        intake.intakeMid()
                                )
                        ),
                        /*new ParallelAction(
                                specimenTwo,
                                intake.intakeDown()
                        ),*/
                        intake.claw(),
                        sleep(0.15),
                        new ParallelAction(
                                lift.liftUpMid(),
                                new SequentialAction(
                                        sleep(0.15),
                                        new ParallelAction(
                                                intake.intakeUp(),
                                                toSubmersibleTwo
                                        )
                                )
                        ),
                        //lift.liftDown(),
                        new ParallelAction(
                                slowMove2,
                                new SequentialAction(
                                        sleep(0.15),
                                        lift.liftDown()
                                )
                        ),
                        //three
                        new ParallelAction(
                                lift.liftDown(),
                                specimenThree,
                                new SequentialAction(
                                        sleep(1.0),
                                        intake.intakeDown()
                                )
                        ),
                        intake.claw(),
                        sleep(0.2),
                        new ParallelAction(
                                intake.intakeUp(),
                                toSubmersibleThree,
                                new SequentialAction(
                                        sleep(0.5),
                                        lift.liftUp()
                                )
                        ),
                        //lift.liftDown(),
                        new ParallelAction(
                                slowMove3,
                                new SequentialAction(
                                        sleep(0.15),
                                        lift.liftDown()
                                )
                        ),
                        //four
                        new ParallelAction(
                                lift.liftDown(),
                                specimenFour,
                                new SequentialAction(
                                        sleep(1.0),
                                        intake.intakeDown()
                                )
                        ),
                        intake.claw(),
                        sleep(0.2),
                        new ParallelAction(
                                intake.intakeUp(),
                                toSubmersibleFour,
                                lift.liftUp()
                        ),
                        //lift.liftDown(),
                        new ParallelAction(
                                slowMove4,
                                new SequentialAction(
                                        sleep(0.15),
                                        lift.liftDown()
                                )
                        ),
                        //park
                        new ParallelAction(
                                toPark,
                                lift.liftDown(),
                                new SequentialAction(
                                        sleep(1.0),
                                        intake.intakeMid()
                                )
                        )
                )
        );
    }
}
