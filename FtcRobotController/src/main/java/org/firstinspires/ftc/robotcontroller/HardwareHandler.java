package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareHandler {
    private final HardwareMap juyoungHardwareMap;
    private final DcMotor leftFront;
    private final DcMotor leftRear;
    private final DcMotor rightFront;
    private final DcMotor rightRear;
    public static double VLF = 1, VRF = 1;

    public HardwareHandler(HardwareMap juyoungHardwareMap) {

        this.juyoungHardwareMap = juyoungHardwareMap;

        leftFront = juyoungHardwareMap.dcMotor.get("leftFront");
        leftRear = juyoungHardwareMap.dcMotor.get("leftRear");
        rightFront = juyoungHardwareMap.dcMotor.get("rightFront");
        rightRear = juyoungHardwareMap.dcMotor.get("rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void moveWithPower(double d, double r, double speed){
        double total = Math.abs(d) + Math.abs(r);
        if (d == 0 && r == 0) {
            leftFront.setPower(0);
            rightFront.setPower(0);
        } else {
            leftFront.setPower((-d + r) / total * speed * VLF);
            rightFront.setPower((-d - r) / total * speed * VRF);
        }
    }
}
