package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.imu.SimpsonIntegrator;

//tape measuere aiming, extension continous, two intake servos

import java.util.HashMap;

public class HardwareHandler {
    private final HardwareMap juyoungHardwareMap;
    private final DcMotor leftFront;
    private final DcMotor leftRear;
    private final DcMotor rightFront;
    private final DcMotor rightRear;
    private final DcMotor linearLiftLeft;
    private final DcMotor linearLiftRight;
    private final DcMotor climbOne;
    private final DcMotor climbTwo;
    public static double VLF = 1, VRF = 1, VLR = 1, VRR = 1;
    private Telemetry telemetry;

    //private BNO055IMU imu;  // IMU sensor object
    private Orientation angles;
    private final SimpsonIntegrator integrator;

    private final int msPollInterval = 100;
    //private BHI260AP imu;

    private IMU imu;

    private DcMotor.RunMode currRunMode;

    private YawPitchRollAngles robotOrientation;


    private final CRServo leftIntake;
    private final CRServo rightIntake;

    private final Servo leftTapeMeasureAim;
    private final Servo rightTapeMeasureAim;

    private final CRServo leftTapeMeasure;
    private final CRServo rightTapeMeasure;

    private final Servo leftLiftAngle;

    private final Servo rightLiftAngle;

    private final Servo leftClaw;
    private final Servo rightClaw;

    private int leftLiftPos = 0;
    private int rightLiftPos = 0;

    public HardwareHandler(HardwareMap juyoungHardwareMap, Telemetry telemetry) {

        this.juyoungHardwareMap = juyoungHardwareMap;

        leftFront = juyoungHardwareMap.dcMotor.get("leftFront");
        leftRear = juyoungHardwareMap.dcMotor.get("leftRear");
        rightFront = juyoungHardwareMap.dcMotor.get("rightFront");
        rightRear = juyoungHardwareMap.dcMotor.get("rightRear");
        linearLiftLeft = juyoungHardwareMap.dcMotor.get("linearLeft");
        linearLiftRight = juyoungHardwareMap.dcMotor.get("linearRight");
        climbOne = juyoungHardwareMap.dcMotor.get("climbOne");
        climbTwo = juyoungHardwareMap.dcMotor.get("climbTwo");
        leftIntake = juyoungHardwareMap.crservo.get("leftIntake");
        rightIntake = juyoungHardwareMap.crservo.get("rightIntake");
        leftTapeMeasureAim = juyoungHardwareMap.servo.get("leftTapeMeasureAim");
        rightTapeMeasureAim = juyoungHardwareMap.servo.get("rightTapeMeasureAim");
        leftTapeMeasure = juyoungHardwareMap.crservo.get("leftTapeMeasure");
        rightTapeMeasure = juyoungHardwareMap.crservo.get("rightTapeMeasure");
        leftLiftAngle = juyoungHardwareMap.servo.get("leftLiftAngle");
        rightLiftAngle = juyoungHardwareMap.servo.get("rightLiftAngle");
        leftClaw = juyoungHardwareMap.servo.get("leftClaw");
        rightClaw = juyoungHardwareMap.servo.get("rightClaw");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        linearLiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        climbOne.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearLiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearLiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        integrator = new SimpsonIntegrator(msPollInterval);
        imu = juyoungHardwareMap.get(IMU.class,"imu");
        // Initialize IMU hardware
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(myIMUparameters);

        this.telemetry = telemetry;

    }

    public void moveWithPower(double d, double r, double s, double speed) { // d : linear movement, r : rotational movement, s : speed (0-1); r is signed with CCW as positive
        //assert (speed <= 1 && speed >= 0): "Speed must be between 0 and 1";
        if (currRunMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            speed = Math.abs(speed);

        double total = Math.abs(d) + Math.abs(r) + Math.abs(s);
        if (d == 0 && r == 0 && s == 0) {
            leftFront.setPower(0);
            leftRear.setPower(0);

            rightFront.setPower(0);
            rightRear.setPower(0);
        }
        else {
            leftFront.setPower((-d + r + s) / total * speed * VLF);
            leftRear.setPower((-d + r - s) / total * speed * VLR); // test to change these values
            rightFront.setPower((-d - r - s) / total * speed * VRF);
            rightRear.setPower((-d - r + s) / total * speed * VRR);

            /*
            d + r + s
            d + r - s
            d - r - s
            d - r + s
             */
        }
    }
   // public boolean isIMUCalibrated() {
        //return imu;
    //}


    public double[] getIMUAngles() {
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll = robotOrientation.getRoll(AngleUnit.DEGREES);
        double[] values = new double[]{Yaw,Pitch,Roll};

        return values;

    }

    public double getAverageEncoderPosition() {
        double lf = leftFront.getCurrentPosition();
        double lr = leftRear.getCurrentPosition();
        double rf = rightFront.getCurrentPosition();
        double rr = rightRear.getCurrentPosition();

        return (lf + lr + rf + rr) / 4.0;
    }

    public void telemetryEncoderPosition() {
        telemetry.addData("leftFront: ", leftFront.getCurrentPosition());
        telemetry.addData("leftRear: ", leftRear.getCurrentPosition());
        telemetry.addData("left diff: ", leftFront.getCurrentPosition()/(leftRear.getCurrentPosition()+0.0001));
        telemetry.addData("rightFront: ", rightFront.getCurrentPosition());
        telemetry.addData("rightRear: ", rightRear.getCurrentPosition());
        telemetry.addData("right diff: ", rightFront.getCurrentPosition()/(rightRear.getCurrentPosition()+0.0001));
        telemetry.addData("Average: ", getAverageEncoderPosition());
        telemetry.update();
    }

    public void moveFourWheel(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    public void moveFourEncoder(double forwardPower, double correction, double strafePower, double overallPower) {
        leftFront.setPower(forwardPower - correction);
        leftRear.setPower(forwardPower - correction);
        rightFront.setPower(forwardPower + correction);
        rightRear.setPower(forwardPower + correction);
    }


    public double distanceToTicks(double distanceInInches) {
        double wheelDiameter = 4.0;
        double wheelCircumference = wheelDiameter * Math.PI;
        double countsPerRevolution = 1120;
        double gearRatio = 1.0;

        return (distanceInInches / wheelCircumference) * countsPerRevolution * gearRatio;
    }

    public void climbOn(double power) {climbOne.setPower(power); }

    public void climbTw(double power) {climbTwo.setPower(power); }

    public void strafeFourWheel(double power, boolean direction) {
        if (direction) {
            leftFront.setPower(-power*0.6);
            leftRear.setPower(power*0.5);
            rightFront.setPower(power*0.5);
            rightRear.setPower(-power*0.6);
        }
        else {
            leftFront.setPower(power*0.6);
            leftRear.setPower(-power*0.5);
            rightFront.setPower(power*0.5);
            rightRear.setPower(-power*0.625 );
        }


    }

    public void stopMotors() {
        moveWithPower(0, 0, 0, 0);  // Stop movement
    }


    public void rotateToAngle(double targetAngle, double power) {
        double[] imuAngles = getIMUAngles();
        double currentYaw = imuAngles[0];
        double tolerance = 1.0;
        double direction = targetAngle > currentYaw ? -1 : 1;
        while (Math.abs(currentYaw - targetAngle) > tolerance) {
            telemetry.addData("curr yaw", currentYaw);
            telemetry.addData("direction", direction);
            telemetry.update();
            double error = targetAngle - currentYaw;
            double turnPower = direction * power * (error / 180.0);
            moveWithPower(0, turnPower, 0, turnPower);
            imuAngles = getIMUAngles();
            currentYaw = imuAngles[0];

        }

        stopMotors();
    }

    public void moveForwardWithHeading(double power, double distance, double desiredHeading) {
        double[] imuAngles;
        double currentYaw;
        double correction;
        double error;
        double initialEncoderPosition = getAverageEncoderPosition();
        double targetPosition = initialEncoderPosition + distanceToTicks(distance);
        long startTime = System.currentTimeMillis();
        long timeOut = 5000;
        imuAngles = getIMUAngles();
        currentYaw = imuAngles[0];
        desiredHeading += currentYaw;

        while (getAverageEncoderPosition() < targetPosition && System.currentTimeMillis() - startTime < timeOut) {
            imuAngles = getIMUAngles();
            currentYaw = imuAngles[0];
            error = desiredHeading - currentYaw;
            correction = error * 0.01;
            moveFourEncoder(power, correction, 0, power);

            telemetry.addData("Encoder Position", getAverageEncoderPosition());
            telemetry.addData("Current Yaw", currentYaw);
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
        }

        stopMotors();
        telemetry.addData("Status", "Movement Complete");
        telemetry.update();
    }

    public void moveBackwardWithHeading(double power, double distance, double desiredHeading) {
        double[] imuAngles;
        double currentYaw;
        double correction;
        double error;
        double initialEncoderPosition = getAverageEncoderPosition();
        double targetPosition = initialEncoderPosition + distanceToTicks(distance);
        long startTime = System.currentTimeMillis();
        long timeOut = 5000;
        imuAngles = getIMUAngles();
        currentYaw = imuAngles[0];
        desiredHeading += currentYaw;

        while (getAverageEncoderPosition() > targetPosition && System.currentTimeMillis() - startTime < timeOut) {
            imuAngles = getIMUAngles();
            currentYaw = imuAngles[0];
            error = desiredHeading - currentYaw;
            correction = error * 0.01;
            moveFourEncoder(power, correction, 0, power);

            telemetry.addData("Encoder Position", getAverageEncoderPosition());
            telemetry.addData("Current Yaw", currentYaw);
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
        }

        stopMotors();
        telemetry.addData("Status", "Movement Complete");
        telemetry.update();
    }

    public void setClaw(String pos) {
        if (pos.equals("open")) {
            leftClaw.setPosition(0.8289);
            rightClaw.setPosition(0.175);
        } else {
            leftClaw.setPosition(0.98); //0.99
            rightClaw.setPosition(0.0228); //0.0128
        }

    }

    public void intakeSystem(double a) {
        leftIntake.setPower(-a);
        rightIntake.setPower(a);
    }

    public void setMeasure(double angle) {
        leftTapeMeasureAim.setPosition(.9728-angle);
        rightTapeMeasureAim.setPosition(angle);
    }

    public void measureAngle(double direction) {
        double leftCurrPos = leftTapeMeasureAim.getPosition();
        double rightCurrPos = rightTapeMeasureAim.getPosition();
        if ((leftCurrPos > 0.7 && direction < 0) || (rightCurrPos > 0 && direction > 0)) {
            leftTapeMeasureAim.setPosition(leftCurrPos + direction * 0.001);
            rightTapeMeasureAim.setPosition(rightCurrPos + direction * -0.001);
        }
    }

    public void measurePosition() {
        telemetry.addData("Left Tape Measure Servo Position", leftTapeMeasureAim.getPosition());
        telemetry.addData("Right Tape Measure Servo Position", rightTapeMeasureAim.getPosition());
    }

    public void launchMeasure(double direction) {
        leftTapeMeasure.setPower(-direction);
        rightTapeMeasure.setPower(direction);
    }

    public void intakeAngle(double angle) {
        leftLiftAngle.setPosition(0.01+angle);
        rightLiftAngle.setPosition(1-angle);
    }

    public void setLiftPos(int pos) {
        leftLiftPos = linearLiftLeft.getCurrentPosition() + pos;
        rightLiftPos = linearLiftRight.getCurrentPosition() + pos;
    }

    public void joystickLift(double power) {
        linearLiftLeft.setPower(power*0.80);
        linearLiftRight.setPower(power*0.80);
        leftLiftPos = linearLiftLeft.getCurrentPosition();
        rightLiftPos = linearLiftRight.getCurrentPosition();
    }

    public void updateLift() {
        int leftDiff = leftLiftPos - linearLiftLeft.getCurrentPosition();
        int rightDiff = rightLiftPos - linearLiftRight.getCurrentPosition();
        if (leftDiff > 200 && rightDiff > 200) {
            linearLiftLeft.setPower(0.8);
            linearLiftRight.setPower(0.8);
        } else if (leftDiff > 0 && rightDiff > 0) {
            linearLiftLeft.setPower(0.75 * (leftDiff / 200.0) + 0.05);
            linearLiftRight.setPower(0.75 * (leftDiff / 200.0) + 0.05);
        } else {
            linearLiftLeft.setPower(0);
            linearLiftRight.setPower(0);
        }
    }

    public void slideTelemetry() {
        telemetry.addData("left slide", leftLiftPos);
        telemetry.addData("right slide", rightLiftPos);
    }

    public void holdLift(boolean button, int pos, double power) {
        if (button) {
            /*telemetry.addData("left", linearLiftLeft.getCurrentPosition());
            telemetry.addData("right", linearLiftRight.getCurrentPosition());
            telemetry.update();*/

            linearLiftLeft.setTargetPosition(pos);
            linearLiftRight.setTargetPosition(-pos);

            linearLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            linearLiftLeft.setPower(power);
            linearLiftRight.setPower(-power);

            while (linearLiftRight.isBusy() && linearLiftLeft.isBusy()) {
                telemetry.addData("left", linearLiftLeft.getCurrentPosition());
                telemetry.addData("right", linearLiftRight.getCurrentPosition());
                telemetry.update();
            }

            linearLiftLeft.setPower(0);
            linearLiftRight.setPower(0);

            linearLiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            linearLiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            /*while(pos > linearLiftLeft.getCurrentPosition() && -pos < linearLiftRight.getCurrentPosition()) {
                telemetry.addData("left", linearLiftLeft.getCurrentPosition());
                telemetry.addData("right", linearLiftRight.getCurrentPosition());
                telemetry.update();
            }*/
        } else {
            linearLiftLeft.setPower(power);
            linearLiftRight.setPower(-power);
        }
    }

    /*private double servoAngle = 0.0;   // Current servo angle (degrees)
    private double angleStep = 5.0;    // Increment angle by 5 degrees (adjustable)
    private boolean prevToggleState = false; // Tracks the previous state of the toggle button

    // Method to toggle the servo angle through a range
    public void toggleServo(boolean toggleButton) {
        // Check if toggle button was just pressed
        if (toggleButton && !prevToggleState) {
            // Increment the servo angle by the step size
            servoAngle += angleStep;

            // Wrap the angle back to 0 if it exceeds 180 degrees
            if (servoAngle > 180) {
                servoAngle = 0;
            }


            //tapeMeasureAim.setPosition(servoAngle / 270.0);
        }

        // Update the previous button state
        prevToggleState = toggleButton;
    }*/

}
