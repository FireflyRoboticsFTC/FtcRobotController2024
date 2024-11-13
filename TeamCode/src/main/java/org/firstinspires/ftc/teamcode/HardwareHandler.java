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


    private final CRServo intake;

    private final Servo tapeMeasureAim;

    private final CRServo tapeMeasure;

    private final Servo leftLiftAngle;

    private final Servo rightLiftAngle;



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
        intake = juyoungHardwareMap.crservo.get("intake");
        tapeMeasureAim = juyoungHardwareMap.servo.get("tapeMeasureAim");
        tapeMeasure = juyoungHardwareMap.crservo.get("tapeMeasure");
        leftLiftAngle = juyoungHardwareMap.servo.get("leftLiftAngle");
        rightLiftAngle = juyoungHardwareMap.servo.get("rightLiftAngle");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        integrator = new SimpsonIntegrator(msPollInterval);
        imu = juyoungHardwareMap.get(IMU.class,"imu");
        // Initialize IMU hardware
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                90,
                                0,
                                0,
                                0)


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
        telemetry.addData("rightFront: ", rightFront.getCurrentPosition());
        telemetry.addData("rightRear: ", rightRear.getCurrentPosition());
        telemetry.addData("Average: ", getAverageEncoderPosition());
        telemetry.update();
    }

    public void moveFourWheel(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    public void climbOn(double power) {
        climbOne.setPower(power);
    }

    public void climbTw(double power) {
        climbTwo.setPower(power);
    }

    public void strafeFourWheel(double power, boolean direction) {
        if (direction) {
            leftFront.setPower(-power);
            leftRear.setPower(power*0.5);
            rightFront.setPower(power);
            rightRear.setPower(-power*0.5);
        }
        else {
            leftFront.setPower(power);
            leftRear.setPower(-power*0.5);
            rightFront.setPower(power);
            rightRear.setPower(-power*0.5 );
        }


    }

    public void stopMotors() {
        moveWithPower(0, 0, 0, 0);  // Stop movement
    }


    public void rotateToAngle(double targetAngle, double power) {
        double[] imuAngles = getIMUAngles();
        double currentYaw = imuAngles[0];
        double tolerance = 1.0;
        double direction = targetAngle > currentYaw ? 1 : -1;
        while (Math.abs(currentYaw - targetAngle) > tolerance) {
            double error = targetAngle - currentYaw;
            double turnPower = direction * power * (error / 180.0);
            moveWithPower(0, turnPower, 0, Math.abs(turnPower));
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
        double targetPosition = initialEncoderPosition + distance;

        while (getAverageEncoderPosition() < targetPosition) {
            imuAngles = getIMUAngles();
            currentYaw = imuAngles[0];
            error = desiredHeading - currentYaw;
            correction = error * 0.01;
            moveWithPower(power, correction, 0, power);
            telemetry.addData("Encoder Position and current yaw","current position " + getAverageEncoderPosition() + "current angle " + currentYaw );
            telemetry.update();
        }

        stopMotors();
    }


    boolean buttonPressed = false;
    ElapsedTime runtime = new ElapsedTime();

    public void toggleSlide(boolean a, double power) {
        // Check if the button is pressed
        boolean isButtonPressed = a; // Change "a" to the desired button
        double rightPowerModifer = 1;
        if (power > 0)
            rightPowerModifer = 1;
        // Check if the button state has changed
        if (isButtonPressed && !buttonPressed) {

            climbOne.setPower(power*rightPowerModifer);
            runtime.reset();

        } else if (!isButtonPressed && buttonPressed) {

            climbOne.setPower(0.0);

        }

        // Update the button state
        buttonPressed = isButtonPressed;

    }

    public void toggleSlideTwo(boolean a, double power) {
        // Check if the button is pressed
        boolean isButtonPressed = a; // Change "a" to the desired button
        double rightPowerModifer = 1;
        if (power > 0)
            rightPowerModifer = 1;
        // Check if the button state has changed
        if (isButtonPressed && !buttonPressed) {

            climbTwo.setPower(power*rightPowerModifer);
            runtime.reset();

        } else if (!isButtonPressed && buttonPressed) {

            climbTwo.setPower(0.0);

        }

        // Update the button state
        buttonPressed = isButtonPressed;

    }


    boolean buttonPressedY = false;
    ElapsedTime runtimeB = new ElapsedTime();

    public void intakeSystem(double a) {
            intake.setPower(a);

    }

    public void measureAngle(double angle) {
            tapeMeasureAim.setPosition(angle);
    }

    public void launchMeasure(double speed){
            tapeMeasure.setPower(speed);
    }

    public void intakeAngle(double angle) {
            leftLiftAngle.setPosition(angle);
            rightLiftAngle.setPosition(1-angle);
    }

    public void toggleLift(boolean y, double power) {
        // Check if the button is pressed
        boolean isButtonPressedY = y; // Change "a" to the desired button
        double powerModifer = 1;
        if (power > 0)
            powerModifer = 0.90;
        // Check if the button state has changed
        if (isButtonPressedY && !buttonPressedY) {

            linearLiftLeft.setPower(-power*powerModifer);
            linearLiftRight.setPower(power);
            runtimeB.reset();

        } else if (!isButtonPressedY && buttonPressedY) {

            linearLiftRight.setPower(0.4);
            linearLiftLeft.setPower(0.4*powerModifer);

        }

        // Update the button state
        buttonPressedY = isButtonPressedY;

    }




}
