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
    public static double dropX = 50.8-0.6+1.5; //you can change these dynamically in ftc dashboard, change the number then initalize
    public static double dropY = 52.8-0.6+1.5; //website http://192.168.43.1:8080/dash

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
            private double leftTargetPos = 1000; // change these two numbers if you want slides to go higher
            private double rightTargetPos = 1000;
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
                    linearLiftLeft.setPower(0.05);  //dont mess with this it keeps the slides from falling
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
                if (runtime.milliseconds() < 400) {
                    return true;
                } else if (runtime.milliseconds() < 1000) { //change this if slides dont go down enough
                    linearLiftLeft.setPower(-1);
                    linearLiftRight.setPower(-1);
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
            leftLiftAngle.setPosition(0.01);
            rightLiftAngle.setPosition(1);
            leftClaw.setPosition(0.99);
            rightClaw.setPosition(0.0128);
            leftTapeMeasureAim.setPosition(.9728-0.05);
            rightTapeMeasureAim.setPosition(0.05);
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
                leftLiftAngle.setPosition(0.01+0.39);
                rightLiftAngle.setPosition(1-0.39);
                leftClaw.setPosition(0.9); //0.97 numbers if you decide to keep claw closed
                rightClaw.setPosition(0.1); //0.0328
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

        public Action clawClose() { return new ClawClose(); }

        public Action clawOpen() { return new ClawOpen(); }
    }

    public Action sleep(double dt) { return new SleepAction(dt); }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(30, 65, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        Intake intake = new Intake(hardwareMap);
        // make a Lift instance
        Lift lift = new Lift(hardwareMap);

        Action toBasket = drive.actionBuilder(initialPose) // the strafe and turn could be combined, idk if i would suggest using a spline
                .strafeTo(new Vector2d(30, dropY+0.3))
                .strafeToLinearHeading(new Vector2d(dropX+0.3, dropY+0.3), Math.toRadians(50))
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
                .strafeTo(new Vector2d(44.5, 42), new TranslationalVelConstraint(8)) //new TranslationalVelConstraint(5.0) makes it have max velocity of 5
                .build();

        Action toBasket2 = drive.actionBuilder(new Pose2d(44.5, 42, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(dropX, dropY), Math.toRadians(45))
                .build();

        Action toBlock2 = drive.actionBuilder(new Pose2d(dropX, dropY, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(54.5, dropY), Math.toRadians(-90))
                .strafeTo(new Vector2d(54.5, 49))
                .strafeTo(new Vector2d(54.5, 42), new TranslationalVelConstraint(8.0))
                .build();

        Action toBasket3 = drive.actionBuilder(new Pose2d(54.5, 42, Math.toRadians(-90)))
                .turn(Math.toRadians(-90-46))
                .strafeToLinearHeading(new Vector2d(dropX-0.3, dropY-0.3), Math.toRadians(45))
                .build();

        //the two block 3 movements can definitely be combined
        Action toBlock3 = drive.actionBuilder(new Pose2d(dropX-0.3, dropY-0.3, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(42, 25), Math.toRadians(0))
                .strafeTo(new Vector2d(47, 25))
                .build();

        Action toBlock3two = drive.actionBuilder(new Pose2d(42, 28, Math.toRadians(0)))
                .strafeTo(new Vector2d(47, 25))
                .build();

        Action toBasket4 = drive.actionBuilder(new Pose2d(47, 25, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(dropX+1.5, dropY+1.5), Math.toRadians(45))
                .build();

        //the two deposit actions can be combined now that the slides dont fall by themselves
        Action deposit4one = drive.actionBuilder(new Pose2d(dropX, dropY, Math.toRadians(45)))
                .strafeTo(new Vector2d(dropX+1.5, dropY+1.5))
                .build();
        Action deposit4two = drive.actionBuilder(new Pose2d(dropX+1.5, dropY+1.5, Math.toRadians(45)))
                .strafeTo(new Vector2d(dropX, dropY))
                .build();

        //change this completely
        Action toPark = drive.actionBuilder(new Pose2d(dropX+1.5, dropY+1.5, Math.toRadians(45)))
                //.turnTo(Math.PI)
                //.waitSeconds(15)
                .strafeToLinearHeading(new Vector2d(26.5, 12), Math.PI, new TranslationalVelConstraint(20.0))
                .strafeTo(new Vector2d(10, 12), new TranslationalVelConstraint(20.0))
                .build();

        intake.servoStart();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                toBasket,
                                new SequentialAction(
                                        sleep(1.2), //when using sleep always add your decimals so it doesnt default to the other sleep
                                        lift.liftUp()
                                )
                        ),
                        intake.intakeOut(),
                        sleep(0.4),
                        //back,
                        new ParallelAction( //block 1
                                toBlock1,
                                lift.liftDown(),
                                new SequentialAction(
                                        sleep(1.2),
                                        intake.intakeDown()
                                )
                        ),
                        intake.clawClose(),
                        sleep(0.3),
                        new ParallelAction(
                                toBasket2,
                                intake.intakeUp(),
                                new SequentialAction(
                                        sleep(0.7),
                                        lift.liftUp()
                                )
                        ),
                        //deposit,
                        intake.intakeOut(),
                        sleep(0.4),
                        //back,
                        new ParallelAction( //block 2
                                toBlock2,
                                lift.liftDown(),
                                new SequentialAction(
                                        sleep(1.5),
                                        intake.intakeDown()
                                )
                        ),
                        intake.clawClose(),
                        sleep(0.3),
                        new ParallelAction(
                                toBasket3,
                                intake.intakeUp(),
                                new SequentialAction(
                                        sleep(1.0),
                                        lift.liftUp()
                                )
                        ),
                        //deposit,
                        intake.intakeOut(),
                        sleep(0.4),
                        //back,
                        lift.liftDown(),
                        new ParallelAction(
                                toBlock3,
                                lift.liftDown(),
                                new SequentialAction(
                                        sleep(1.5),
                                        intake.intakeDown()
                                )
                        ),
                        intake.clawClose(),
                        sleep(0.3),
                        new ParallelAction(
                                toBasket4,
                                intake.intakeUp(),
                                new SequentialAction(
                                        sleep(1.3),
                                        lift.liftUp()
                                )
                        ),
                        //deposit
                        intake.clawOpen(),
                        sleep(0.4),
                        //park
                        toPark
                        /*new ParallelAction(
                                //lift.climb(),
                                toPark,
                                lift.liftDown()
                        )*/
                )
        );
    }
}
