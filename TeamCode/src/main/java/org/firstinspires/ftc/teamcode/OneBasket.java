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
        private final DcMotor climbOne;

        public Lift(HardwareMap hardwareMap) {
            linearLiftLeft = hardwareMap.dcMotor.get("linearLeft");
            linearLiftRight = hardwareMap.dcMotor.get("linearRight");
            climbOne = hardwareMap.dcMotor.get("climbOne");

            linearLiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
            climbOne.setDirection(DcMotorSimple.Direction.REVERSE);

            linearLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            climbOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            linearLiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            linearLiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            climbOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            linearLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linearLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            climbOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
                if (leftPos < 1590 && rightPos < 1590) {
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

        public class Climb implements Action {
            private boolean initialized = false;
            private ElapsedTime runtime = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    climbOne.setPower(1);
                    initialized = true;
                    runtime.reset();
                }

                // checks lift's current position
                if (runtime.milliseconds() < 5000) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    climbOne.setPower(0);
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

        public Action climb() {return new Climb(); }
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

        public class ClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftClaw.setPosition(0.99);
                rightClaw.setPosition(0.0128);
                return false;
            }
        }

        public class ClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftClaw.setPosition(0.8289);
                rightClaw.setPosition(0.175);
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

        public Action clawClose() { return new ClawClose(); }

        public Action clawOpen() { return new ClawOpen(); }
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
        double dropX = 50.8-0.6;
        double dropY = 52.8-0.6;

        Action toBasket = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(30, dropY+0.3))
                .strafeTo(new Vector2d(dropX+0.3, dropY+0.3))
                .turnTo(Math.toRadians(50))
                .build();

        /*Action deposit = drive.actionBuilder(new Pose2d(51, 53, Math.toRadians(50)))
                .strafeTo(new Vector2d(55, 57))
                .build();

        Action back = drive.actionBuilder(new Pose2d(55, 57, Math.toRadians(50)))
                .strafeTo(new Vector2d(51, 53))
                .build();*/

        Action toBlock1 = drive.actionBuilder(new Pose2d(dropX+0.3, dropY+0.3, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(44.5, dropY+0.3), Math.toRadians(-90))
                .strafeTo(new Vector2d(44.5, 49))
                .strafeTo(new Vector2d(44.5, 42), new TranslationalVelConstraint(5.0))
                .build();

        Action toBasket2 = drive.actionBuilder(new Pose2d(44.5, 42, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(dropX, dropY), Math.toRadians(45))
                .build();

        Action toBlock2 = drive.actionBuilder(new Pose2d(dropX, dropY, Math.toRadians(45)))
                .turn(Math.toRadians(91))
                .strafeToLinearHeading(new Vector2d(54.5, dropY), Math.toRadians(-90))
                .strafeTo(new Vector2d(54.5, 49))
                .strafeTo(new Vector2d(54.5, 42), new TranslationalVelConstraint(5.0))
                .build();

        Action toBasket3 = drive.actionBuilder(new Pose2d(54.5, 42, Math.toRadians(-90)))
                .turn(Math.toRadians(-90-46))
                .strafeToLinearHeading(new Vector2d(dropX-0.3, dropY-0.3), Math.toRadians(45))
                .build();

        //this is where i would put code for block 4 if uh i had code for grabbing block 4
        Action toBlock3one = drive.actionBuilder(new Pose2d(dropX-0.3, dropY-0.3, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(42, 25), Math.toRadians(0))
                .build();

        Action toBlock3two = drive.actionBuilder(new Pose2d(42, 28, Math.toRadians(0)))
                .strafeTo(new Vector2d(47, 25))
                .build();

        Action toBasket4 = drive.actionBuilder(new Pose2d(47, 25, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(dropX, dropY), Math.toRadians(45))
                .build();

        Action deposit4one = drive.actionBuilder(new Pose2d(dropX, dropY, Math.toRadians(45)))
                .strafeTo(new Vector2d(dropX+1.5, dropY+1.5))
                .build();

        Action deposit4two = drive.actionBuilder(new Pose2d(dropX+1.5, dropY+1.5, Math.toRadians(45)))
                .strafeTo(new Vector2d(dropX, dropY))
                .build();

        Action toPark = drive.actionBuilder(new Pose2d(dropX, dropY, Math.toRadians(45)))
                .turnTo(0)
                //.waitSeconds(15)
                .strafeTo(new Vector2d(26.5, 12), new TranslationalVelConstraint(20.0))
                .strafeTo(new Vector2d(10, 12), new TranslationalVelConstraint(20.0))
                .build();

        Action sleep = new SleepAction(0.4);

        Action sleep2 = new SleepAction(0.4);

        Action sleep3 = new SleepAction(0.4);

        Action sleep4 = new SleepAction(0.6);

        Action sleep5 = new SleepAction(0.3);

        intake.servoStart();

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
                        new ParallelAction(
                                toBlock3one,
                                lift.liftDown(),
                                intake.intakeDown()
                        ),
                        new ParallelAction(
                                toBlock3two,
                                intake.clawOpen()
                        ),
                        intake.clawClose(),
                        sleep4,
                        new ParallelAction(
                                toBasket4,
                                intake.intakeUp()
                        ),
                        lift.liftUp(),
                        //deposit
                        deposit4one,
                        intake.clawOpen(),
                        sleep5,
                        lift.liftUp(),
                        deposit4two,
                        //park
                        new ParallelAction(
                                //lift.climb(),
                                toPark,
                                lift.liftDown()
                        )
                )
        );
    }
}
