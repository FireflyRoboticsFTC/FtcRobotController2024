package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
@Autonomous(name = "rr test auto 3", group = "Autonomous")
public class ThreeSpecimein extends LinearOpMode {

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
                    linearLiftLeft.setPower(0.61);
                    linearLiftRight.setPower(0.61);
                    initialized = true;
                }

                // checks lift's current position
                double leftPos = linearLiftLeft.getCurrentPosition();
                double rightPos = linearLiftRight.getCurrentPosition();
                packet.put("liftPos", leftPos);
                if (leftPos < 1470+85 && rightPos < 1470+85) {
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
            private double leftStartPos = 0;
            private double rightStartPos = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearLiftLeft.setPower(0.35);
                    linearLiftRight.setPower(0.35);
                    leftStartPos = linearLiftLeft.getCurrentPosition();
                    rightStartPos = linearLiftRight.getCurrentPosition();
                    initialized = true;
                }

                // checks lift's current position
                double leftPos = linearLiftLeft.getCurrentPosition() - leftStartPos;
                double rightPos = linearLiftRight.getCurrentPosition() - rightStartPos;
                packet.put("liftPos", leftPos);
                if (leftPos < 1595+45 && rightPos < 1595+45) {
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

        public class LiftUp3 implements Action {
            private boolean initialized = false;
            private double leftStartPos = 0;
            private double rightStartPos = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearLiftLeft.setPower(0.32);
                    linearLiftRight.setPower(0.32);
                    leftStartPos = linearLiftLeft.getCurrentPosition();
                    rightStartPos = linearLiftRight.getCurrentPosition();
                    initialized = true;
                }

                // checks lift's current position
                double leftPos = linearLiftLeft.getCurrentPosition() - leftStartPos;
                double rightPos = linearLiftRight.getCurrentPosition() - rightStartPos;
                packet.put("liftPos", leftPos);
                if (leftPos < 1605+60 && rightPos < 1605+60) {
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

        public Action liftUp() { return new LiftUp(); }

        public Action liftUp2() {
            return new LiftUp2();
        }

        public Action liftUp3() {
            return new LiftUp3();
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
            leftLiftAngle.setPosition(0);
            rightLiftAngle.setPosition(1);
            leftClaw.setPosition(0.99);
            rightClaw.setPosition(0.0128);
        }

        public class TapeMeasure implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftTapeMeasureAim.setPosition(.9728);
                rightTapeMeasureAim.setPosition(0);
                return false;
            }
        }

        public class IntakeDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
                leftLiftAngle.setPosition(0.01+0.27);
                rightLiftAngle.setPosition(1-0.27);
                leftClaw.setPosition(0.98);
                rightClaw.setPosition(0.0228);
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
                leftClaw.setPosition(0.99);
                rightClaw.setPosition(0.0128);
                return false;
            }
        }

        public class IntakeOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
                return false;
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

        public Action intakeDown() {
            return new IntakeDown();
        }

        public Action intakeUp() {
            return new IntakeUp();
        }

        public Action intakeOut() {
            return new IntakeOut();
        }

        public Action tapeMeasure() {
            return new TapeMeasure();
        }

        public Action claw() {
            return new Claw();
        }
    }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-7.5, 65, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        Intake intake = new Intake(hardwareMap);
        // make a Lift instance
        Lift lift = new Lift(hardwareMap);

        Action toSubmersible = drive.actionBuilder(initialPose)
                .lineToY(40.5, new TranslationalVelConstraint(17.0))
                .waitSeconds(0.6)
                .lineToY(48.5, new TranslationalVelConstraint(20.0))
                .build();

        Action toBlock = drive.actionBuilder(new Pose2d(-7.5, 48.5, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-50, 53))
                .build();
                //.setReversed(true)
                //.lineToY(45, new TranslationalVelConstraint(5.0))
                //.build();

        Action intakeBlock = drive.actionBuilder(new Pose2d(-50, 53, Math.toRadians(-90)))
                .setReversed(true)
                .lineToY(45, new TranslationalVelConstraint(8.0))
                .build();

        Action backwards = drive.actionBuilder(new Pose2d(-49.5, 45, Math.toRadians(-90)))
                .lineToYSplineHeading(47.5, Math.toRadians(90))
                .build();

        Action toSpecimen = drive.actionBuilder(new Pose2d(-49.5, 47.5, Math.toRadians(90)))
                .lineToY(44)
                .strafeTo(new Vector2d(-56.5, 44))
                //.strafeTo(new Vector2d(-56.5, 50), new TranslationalVelConstraint(5.0))
                .build();

        Action intakeSpecimen = drive.actionBuilder(new Pose2d(-56.5, 44, Math.toRadians(90)))
                .strafeTo(new Vector2d(-56.5, 50), new TranslationalVelConstraint(5.0))
                .build();

        Action toSubmersible2 = drive.actionBuilder(new Pose2d(-56.5, 50, Math.toRadians(90)))
                .turnTo(-Math.PI / 2)
                .setTangent(0.0)
                .splineToConstantHeading(new Vector2d(-1, 43-0.25), -Math.PI / 2.0)
                .waitSeconds(0.8)
                .lineToY(51, new TranslationalVelConstraint(20.0))
                .build();

        Action backwards2 = drive.actionBuilder(new Pose2d(-1, 51, Math.toRadians(-90)))
                .lineToYSplineHeading(60.8, Math.toRadians(180))
                .build();

        Action toObservationZone = drive.actionBuilder(new Pose2d(-1, 60.8, Math.toRadians(180)))
                .lineToX(-24)
                //.lineToX(-32, new TranslationalVelConstraint(8.0))
                .build();

        Action intakeSpecimen2 = drive.actionBuilder(new Pose2d(-24, 60.8, Math.toRadians(180)))
                .lineToX(-32, new TranslationalVelConstraint(8.0))
                .build();

        Action toSubmersible3 = drive.actionBuilder(new Pose2d(-32, 60.8, Math.toRadians(180)))
                .turnTo(-Math.PI / 2)
                .setTangent(0.0)
                .splineToConstantHeading(new Vector2d(7, 44.5-0.25), -Math.PI / 2.0)
                .waitSeconds(0.8)
                .lineToY(55, new TranslationalVelConstraint(20.0))
                .build();

        Action toPark = drive.actionBuilder(new Pose2d(7, 55, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-42, 65))
                .build();

        Action sleep = new SleepAction(0.6);

        intake.servoStart();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intake.tapeMeasure(),
                                intake.intakeUp(),
                                toSubmersible,
                                lift.liftUp()
                        ),
                        new ParallelAction(
                                toBlock,
                                intake.intakeDown(),
                                lift.liftDown()
                        ),
                        //
                        new ParallelAction(
                                intakeBlock,
                                intake.claw()
                        ),
                        //
                        backwards,
                        intake.intakeOut(),
                        sleep,
                        new ParallelAction(
                                toSpecimen,
                                intake.intakeDown()
                        ),
                        //
                        new ParallelAction(
                                intakeSpecimen,
                                intake.claw()
                        ),
                        //
                        new ParallelAction(
                                intake.intakeUp(),
                                toSubmersible2,
                                lift.liftUp3()
                        ),
                        backwards2,
                        new ParallelAction(
                                lift.liftDown(),
                                toObservationZone,
                                intake.intakeDown()
                        ),
                        //
                        new ParallelAction(
                                intakeSpecimen2,
                                intake.claw()
                        ),
                         //
                        new ParallelAction(
                                intake.intakeUp(),
                                toSubmersible3,
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
