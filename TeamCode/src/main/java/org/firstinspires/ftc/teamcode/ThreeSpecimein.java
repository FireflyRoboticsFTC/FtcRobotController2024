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
                    linearLiftLeft.setPower(0.6);
                    linearLiftRight.setPower(0.6);
                    initialized = true;
                }

                // checks lift's current position
                double leftPos = linearLiftLeft.getCurrentPosition();
                double rightPos = linearLiftRight.getCurrentPosition();
                packet.put("liftPos", leftPos);
                if (leftPos < 1430 && rightPos < 1430) {
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
                if (leftPos < 1480 && rightPos < 1480) {
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
                if (leftPos < 1460 && rightPos < 1460) {
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

        public Intake(HardwareMap hardwareMap) {
            leftIntake = hardwareMap.crservo.get("leftIntake");
            rightIntake = hardwareMap.crservo.get("rightIntake");
            leftLiftAngle = hardwareMap.servo.get("leftLiftAngle");
            rightLiftAngle = hardwareMap.servo.get("rightLiftAngle");
            leftTapeMeasureAim = hardwareMap.servo.get("leftTapeMeasureAim");
            rightTapeMeasureAim = hardwareMap.servo.get("rightTapeMeasureAim");
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
                leftLiftAngle.setPosition(0.01+0.315);
                rightLiftAngle.setPosition(1-0.315);
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

        public class IntakeOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
                return false;
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
                .lineToY(38.5, new TranslationalVelConstraint(17.0))
                .waitSeconds(0.6)
                .lineToY(48, new TranslationalVelConstraint(20.0))
                .build();

        Action toBlock = drive.actionBuilder(new Pose2d(-7.5, 48, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-50, 51))
                .setReversed(true)
                .lineToY(44.5, new TranslationalVelConstraint(5.0))
                .build();

        Action backwards = drive.actionBuilder(new Pose2d(-49.5, 44.5, Math.toRadians(-90)))
                .lineToYSplineHeading(50, Math.toRadians(90))
                .build();

        Action toSpecimen = drive.actionBuilder(new Pose2d(-49.5, 50, Math.toRadians(90)))
                .lineToY(47)
                .strafeTo(new Vector2d(-56.5, 47))
                .strafeTo(new Vector2d(-56.5, 53), new TranslationalVelConstraint(5.0))
                .build();

        Action toSubmersible2 = drive.actionBuilder(new Pose2d(-56.5, 53, Math.toRadians(90)))
                .turnTo(-Math.PI / 2)
                .setTangent(0.0)
                .splineToConstantHeading(new Vector2d(-1, 42), -Math.PI / 2.0)
                .waitSeconds(0.8)
                .lineToY(48, new TranslationalVelConstraint(20.0))
                .build();

        Action backwards2 = drive.actionBuilder(new Pose2d(-1, 48, Math.toRadians(-90)))
                .lineToYSplineHeading(60.8, Math.toRadians(180))
                .build();

        Action toObservationZone = drive.actionBuilder(new Pose2d(-1, 60.8, Math.toRadians(180)))
                .lineToX(-26.5)
                .lineToX(-35.5, new TranslationalVelConstraint(8.0))
                .build();

        Action toSubmersible3 = drive.actionBuilder(new Pose2d(-35.5, 60.8, Math.toRadians(180)))
                .turnTo(-Math.PI / 2)
                .setTangent(0.0)
                .splineToConstantHeading(new Vector2d(6, 43.5), -Math.PI / 2.0)
                .waitSeconds(0.8)
                .lineToY(51, new TranslationalVelConstraint(20.0))
                .build();

        Action toPark = drive.actionBuilder(new Pose2d(6, 50.5, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-42, 65))
                .build();

        Action sleep = new SleepAction(0.6);

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
                        backwards,
                        intake.intakeOut(),
                        sleep,
                        new ParallelAction(
                                toSpecimen,
                                intake.intakeDown()
                        ),
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
