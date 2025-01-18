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
@Autonomous(name = "one basket", group = "Autonomous")
public class OneBasket extends LinearOpMode{
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
                    linearLiftLeft.setPower(0.7);
                    linearLiftRight.setPower(0.7);
                    initialized = true;
                }

                // checks lift's current position
                double leftPos = linearLiftLeft.getCurrentPosition();
                double rightPos = linearLiftRight.getCurrentPosition();
                packet.put("liftPos", leftPos);
                if (leftPos < 1530 && rightPos < 1530) {
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
                if (runtime.milliseconds() < 800) {
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

        public class IntakeOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
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

        public Action intakeOut() {
            return new IntakeOut();
        }

        public Action intakeDown() {
            return new IntakeDown();
        }

        public Action intakeUp() {
            return new IntakeUp();
        }

        public Action tapeMeasure() {
            return new TapeMeasure();
        }
    }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(30, 65, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        Intake intake = new Intake(hardwareMap);
        // make a Lift instance
        Lift lift = new Lift(hardwareMap);
        double dropX = 50.8;
        double dropY = 52.8;

        Action toBasket = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(30, dropY))
                .strafeTo(new Vector2d(dropX, dropY))
                .turnTo(Math.toRadians(50))
                .build();

        /*Action deposit = drive.actionBuilder(new Pose2d(51, 53, Math.toRadians(50)))
                .strafeTo(new Vector2d(55, 57))
                .build();

        Action back = drive.actionBuilder(new Pose2d(55, 57, Math.toRadians(50)))
                .strafeTo(new Vector2d(51, 53))
                .build();*/

        Action toBlock1 = drive.actionBuilder(new Pose2d(dropX, dropY, Math.toRadians(50)))
                .strafeToLinearHeading(new Vector2d(44.5, dropY), Math.toRadians(-90))
                .strafeTo(new Vector2d(44.5, 47))
                .strafeTo(new Vector2d(44.5, 42), new TranslationalVelConstraint(5.0))
                .build();

        Action toBasket2 = drive.actionBuilder(new Pose2d(44.5, 42, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(dropX, dropY), Math.toRadians(50))
                .build();

        Action toBlock2 = drive.actionBuilder(new Pose2d(dropX, dropY, Math.toRadians(50)))
                .strafeToLinearHeading(new Vector2d(54.5, dropY), Math.toRadians(-90))
                .strafeTo(new Vector2d(54.5, 47))
                .strafeTo(new Vector2d(54.5, 42), new TranslationalVelConstraint(5.0))
                .build();

        Action toBasket3 = drive.actionBuilder(new Pose2d(54.5, 42, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(dropX, dropY), Math.toRadians(50))
                .build();

        Action toPark = drive.actionBuilder(new Pose2d(dropX, dropY, Math.toRadians(50)))
                .turnTo(0)
                //.waitSeconds(15)
                .strafeTo(new Vector2d(-61, 60))
                .build();

        Action sleep = new SleepAction(1);

        Action sleep2 = new SleepAction(1);

        Action sleep3 = new SleepAction(1);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction( //preload
                            intake.intakeUp(),
                            intake.tapeMeasure(),
                            toBasket
                        ),
                        lift.liftUp(),
                        intake.intakeOut(),
                        sleep,
                        new ParallelAction(
                                //lift.liftUp(),
                                intake.intakeUp()
                        ),
                        //back,
                        lift.liftDown(),
                        new ParallelAction( //block 1
                                toBlock1,
                                lift.liftDown(),
                                intake.intakeDown()
                        ),
                        new ParallelAction(
                                toBasket2,
                                intake.intakeUp()
                        ),
                        lift.liftUp(),
                        //deposit,
                        intake.intakeOut(),
                        sleep2,
                        new ParallelAction(
                                //lift.liftUp(),
                                intake.intakeUp()
                        ),
                        //back,
                        lift.liftDown(),
                        new ParallelAction( //block 2
                                toBlock2,
                                lift.liftDown(),
                                intake.intakeDown()
                        ),
                        new ParallelAction(
                                toBasket3,
                                intake.intakeUp()
                        ),
                        lift.liftUp(),
                        //deposit,
                        intake.intakeOut(),
                        sleep3,
                        new ParallelAction(
                                //lift.liftUp(),
                                intake.intakeUp()
                        ),
                        //back,
                        lift.liftDown(),
                        new ParallelAction( //park
                                toPark,
                                lift.liftDown()
                        )
                )
        );
    }
}
