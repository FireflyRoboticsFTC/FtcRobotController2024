package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.VSLAMChassis.angle;
import static org.firstinspires.ftc.teamcode.Components.VSLAMChassis.xpos;
import static org.firstinspires.ftc.teamcode.Components.VSLAMChassis.ypos;
import static org.firstinspires.ftc.teamcode.Robot.faked;
import static org.firstinspires.ftc.teamcode.Robot.resetten;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


public class Turret {

    private LinearOpMode op = null;
    private DcMotorEx turret_Rotation = null;
    private DcMotorEx turret_Extension = null;
    private Servo turret_Angle_Control = null;
    private Servo turret_Angle_Control2 = null;
    private Servo basketArmServo = null;
    private Servo basketActuationServo = null;
    private double lastTime=0, lastServoPos=0,servoDist=0;
    private final double DEG_PER_TICK_MOTOR = 18.0/116.0, DEG_PER_TICK_SERVO = 118.0/270.0/35.0, minDiffTime =.3;
    private final double TICKS_PER_INCH = 955.0/32.0;
    private final double MAX_EXTENSION_TICKS = 955;
    private final double MIN_EXTENSION_TICKS = 15;
    private final double MAX_ROTATION_TICKS = 570;
    private double[] lastTimes = {0,0};
    private final double TORQUE_GEAR_RATIO = 10;
    private final double SPEED_GEAR_RATIO = 10;
    private final double ANGLE_CONTROL_SERVO_TOTAL_DEGREES = 35;
    public static double [][][]turret_saved_positions={{{84,-48,0},{84,-48,0}},{{12,-24,15},{12,-24,15}}};

    boolean hardware_present = true;
    boolean servoPos = false;
    boolean servoPos2 = false;
    boolean downCap = false;
    public static double extendPosition=0, rotatePosition=0;
    boolean angleControlling = false, arming = false, basketing = false,areTeleop = false;
    private StateMachine checker = null;


    // initialization of outtakeMotor
    public Turret(LinearOpMode opMode, LedColor led_bank, boolean isTeleOp, StateMachine checkers){
        checker = checkers;
        op = opMode;
        areTeleop = isTeleOp;
        if (hardware_present) {
            turret_Rotation = (DcMotorEx)opMode.hardwareMap.dcMotor.get("turret_Rotation");
            turret_Rotation.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            turret_Extension = (DcMotorEx)opMode.hardwareMap.dcMotor.get("turret_Extension");
            turret_Extension.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            turret_Angle_Control = opMode.hardwareMap.get(Servo.class, "turret_Angle_Control");
            turret_Angle_Control2 = opMode.hardwareMap.get(Servo.class, "turret_Angle_Control2");
            basketArmServo = op.hardwareMap.get(Servo.class, "basketActuationServo");
            basketActuationServo = op.hardwareMap.get(Servo.class, "basketArmServo");
            turret_Rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        turret_Angle_Control.setPosition(0);
        turret_Angle_Control2.setPosition(118.0/270);
        turret_Rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_Rotation.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret_Extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_Extension.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret_Extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        basketActuationServo.setPosition(0.58);
        basketArmServo.setPosition(0.0);
        if(!isTeleOp) {
//            turret_Angle_Control.setPosition(0);
//            turret_Angle_Control2.setPosition(118.0/270);
//            turret_Rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            turret_Rotation.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            turret_Extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            turret_Extension.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            turret_Extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            turret_Rotation.setTargetPosition((int)(-85/DEG_PER_TICK_MOTOR));
//            turret_Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turret_Rotation.setPower(0.5);
//            basketActuationServo.setPosition(0.58);
//            basketArmServo.setPosition(0.0);
        }
        turret_saved_positions[0][1][0] = 80;//-36
        turret_saved_positions[0][1][1] = -53;//-60
        turret_saved_positions[0][1][2] = 0;
        turret_saved_positions[0][0][0] = 80;//-36
        turret_saved_positions[0][0][1] = -53;//-60
        turret_saved_positions[0][0][2] = 0;
        turret_saved_positions[1][1][0] = 12;//-36
        turret_saved_positions[1][1][1] = -24;//-60
        turret_saved_positions[1][1][2] = 15;
        turret_saved_positions[1][0][0] = 12;//-36
        turret_saved_positions[1][0][1] = -24;//-60
        turret_saved_positions[1][0][2] = 15;
        op.sleep(1000);
    }

    public int turret_extension_pos;
    public int turret_rotation_pos;
    public double turret_angle_control_rotations;
    double turret_angle_control_full_rotations; //will update this in teleop loop

    public void updateTurretPositions(){
        extendPosition = turret_Extension.getCurrentPosition();
        rotatePosition = turret_Rotation.getCurrentPosition();
        if(extendPosition<300){
            checker.setState(StateMachine.States.TURRET_SHORT,true);
        }
        else{
            checker.setState(StateMachine.States.TURRET_SHORT,false);
        }
        if (abs(rotatePosition) < 10) {
            checker.setState(StateMachine.States.TURRET_STRAIGHT,true);
        }
        else{
            checker.setState(StateMachine.States.TURRET_STRAIGHT,false);;
        }
        if (extendPosition < 30) {
            checker.setState(StateMachine.States.EXTENDED,false);
        }
        else{
            checker.setState(StateMachine.States.EXTENDED,true);
        }
        if (basketArmServo.getPosition() < 0.05) {
            checker.setState(StateMachine.States.BASKET_ARM_REST,true);
        }
        else{
            checker.setState(StateMachine.States.BASKET_ARM_REST,false);
        }
        if (basketActuationServo.getPosition() > 0.85) {
            checker.setState(StateMachine.States.BASKET_TRANSFER,true);
            checker.setState(StateMachine.States.BASKET_DROP,false);
            checker.setState(StateMachine.States.BASKET_CIELING,false);


        }
        else if (basketActuationServo.getPosition() < 0.45) {
            checker.setState(StateMachine.States.BASKET_TRANSFER,false);
            checker.setState(StateMachine.States.BASKET_DROP,true);
            checker.setState(StateMachine.States.BASKET_CIELING,false);

        }
        else{
            checker.setState(StateMachine.States.BASKET_TRANSFER,false);
            checker.setState(StateMachine.States.BASKET_CIELING,true);
            checker.setState(StateMachine.States.BASKET_DROP,false);


        }
        if(checker.getState(StateMachine.States.EXTENDED)&&!checker.getState(StateMachine.States.RAISED)){
            resetten=true;
            arming = false;
            basketing = false;
            angleControlling = false;
        }
        else{
            resetten=false;
        }
        op.telemetry.addData("extendoPos", extendPosition);
        op.telemetry.addData("rotatePos", rotatePosition);
        op.telemetry.addData("basketDown", checker.getState(StateMachine.States.BASKET_ARM_REST));
        op.telemetry.addData("basketActuationDown", checker.getState(StateMachine.States.BASKET_TRANSFER));

    }
    public void aimHigh(){
        FlipBasketArmToPosition(.45);
        AngleControlRotating(35);
        TurretExtend(17,24,0.8);
        turret_Rotation.setTargetPosition((int) (-65/DEG_PER_TICK_MOTOR));
        turret_Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret_Rotation.setPower(0.5);
        op.sleep(2000);
        FlipBasketArmToPosition(.55);
        FlipBasketToPosition(0.0);
    }
    public void TurretExtend (double height_inches, double extension_inches, double power) {
        double extension_length = Math.sqrt(pow(height_inches, 2) + pow(extension_inches, 2));
        if (extension_length * TICKS_PER_INCH > MAX_EXTENSION_TICKS) {
            turret_Extension.setTargetPosition((int) MAX_EXTENSION_TICKS);
        }
        else {
            turret_Extension.setTargetPosition((int) (extension_length * TICKS_PER_INCH));
        }
        turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret_Extension.setPower(power);
    }

    public void TurretExtendSimple (int extension_inches, double power) {
        if (extension_inches * TICKS_PER_INCH > MAX_EXTENSION_TICKS) {
            turret_Extension.setTargetPosition((int) MAX_EXTENSION_TICKS);
        }
        else if (extension_inches * TICKS_PER_INCH < MIN_EXTENSION_TICKS) {
            turret_Extension.setTargetPosition((int) MIN_EXTENSION_TICKS);
        }
        else {
            turret_Extension.setTargetPosition((int) (extension_inches * TICKS_PER_INCH));
        }
        turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret_Extension.setPower(power);
    }
    public void runTurretWithoutEncoder(){
        turret_Extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret_Rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void TurretRotate (double targetAngle) {
        targetAngle/=DEG_PER_TICK_MOTOR;
        op.telemetry.addData("newAngle",targetAngle);
        int curPos = (int)rotatePosition;
        if(targetAngle>MAX_ROTATION_TICKS){
            targetAngle = MAX_ROTATION_TICKS;
        }
        if(targetAngle<-MAX_ROTATION_TICKS){
            targetAngle = -MAX_ROTATION_TICKS;
        }
        double dist = targetAngle - curPos;
        turret_Rotation.setPower(dist/MAX_ROTATION_TICKS);
    }

    public void BasketArmFlipLowExtend (double power) {
//        basketArmServo.setPosition(190 * TORQUE_GEAR_RATIO);
//
//        turret_Extension.setTargetPosition(1);
//        turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turret_Extension.setPower(power);
    }

    public void BasketArmFlipHighExtend (double power) {
//        basketArmServo.setPosition(115 * TORQUE_GEAR_RATIO);
//
//        turret_Extension.setTargetPosition(1);
//        turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turret_Extension.setPower(power);
    }

    public void TurretAngleControlRotating (double torget_point) {
        double thisTime = op.getRuntime();
        if (torget_point > 1) {
            torget_point = 1;
        }
        else if (torget_point < 0) {
            torget_point = 0;
        }

//        if (basketDown && turret_Angle_Control.getPosition() < 0.1 && torget_point < turret_Angle_Control.getPosition()) {
//            basketActuationServo.setPosition(0.77);
//        }

        if(thisTime-lastTime>minDiffTime) {
            turret_Angle_Control.setPosition(torget_point);
            turret_Angle_Control2.setPosition(118.0/270-torget_point);
            lastTime=thisTime;
            servoDist=Math.abs(torget_point-lastServoPos);
            lastServoPos=torget_point;
        }
//        turret_Angle_Control.setPosition(-.5);
//        turret_Angle_Control2.setPosition(.5);
//        op.telemetry.addData("difference", target_point - turret_Angle_Control.getPosition());

    }
    public void AutoAngleControlRotating (double torget_point) {
        torget_point*=DEG_PER_TICK_SERVO;
        double thisTime = op.getRuntime();
        if (torget_point > 1) {
            torget_point = 1;
        }
        else if (torget_point < 0) {
            torget_point = 0;
        }

        if(thisTime-lastTime>minDiffTime) {
            turret_Angle_Control.setPosition(torget_point);
            turret_Angle_Control2.setPosition(118.0/270-torget_point);
            lastTime=thisTime;
            servoDist=Math.abs(torget_point-lastServoPos);
            lastServoPos=torget_point;
            op.telemetry.addData("lastTime",lastTime);
        }
//        turret_Angle_Control.setPosition(-.5);
//        turret_Angle_Control2.setPosition(.5);
//        op.telemetry.addData("difference", target_point - turret_Angle_Control.getPosition());

    }
    public void AngleControlRotating (double torget_point) {
        torget_point*=DEG_PER_TICK_SERVO;
        if (torget_point > 1) {
            torget_point = 1;
        }
        else if (torget_point < 0) {
            torget_point = 0;
        }
            turret_Angle_Control.setPosition(torget_point);
            turret_Angle_Control2.setPosition(118.0/270-torget_point);
//        turret_Angle_Control.setPosition(-.5);
//        turret_Angle_Control2.setPosition(.5);
//        op.telemetry.addData("difference", target_point - turret_Angle_Control.getPosition());

    }
    public void rotateToPosition(double targetAngle){
        turret_Rotation.setTargetPosition((int) (targetAngle/DEG_PER_TICK_MOTOR));
        turret_Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret_Rotation.setPower(targetAngle);
    }
    public void TurretSlidesToPosition (double x, double y, double z, double power) {
        updateTurretPositions();
        double v = x;
        x=y;
        y=v;
        double rotation_angle = Math.atan2(y,x) * (180 / PI);
        turret_Rotation.setTargetPosition((int) (rotation_angle/DEG_PER_TICK_MOTOR));
        turret_Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret_Rotation.setPower(power);


        double extension_length_flat = Math.sqrt(pow(x, 2) + pow(y, 2));
        if(extension_length_flat ==0){
            extension_length_flat = 0.01;
        }
        double extension_length = Math.sqrt(pow(extension_length_flat, 2) + pow(z, 2));
        if(extension_length_flat*TICKS_PER_INCH<extendPosition){
            extension_length_flat = extendPosition;
        }
        if (extension_length * TICKS_PER_INCH > MAX_EXTENSION_TICKS) {
            turret_Extension.setTargetPosition((int) MAX_EXTENSION_TICKS);
        }
        else {
            turret_Extension.setTargetPosition((int) (extension_length * TICKS_PER_INCH));
        }
        turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret_Extension.setPower(power);


        double elevation_angle = Math.atan2(z,extension_length_flat) * (180 / PI);

        op.telemetry.addData("angle control angle", elevation_angle / ANGLE_CONTROL_SERVO_TOTAL_DEGREES);
        if(elevation_angle<0){
            elevation_angle=0;
        }
        if(elevation_angle>35){
            elevation_angle=35;
        }
        AngleControlRotating(elevation_angle);
    }

    public void TurretManualRotation (double rotation) { //stick_x
        if((rotatePosition > MAX_ROTATION_TICKS && rotation > 0) || (rotatePosition< -MAX_ROTATION_TICKS && rotation < 0)) {
            turret_Rotation.setPower(0);
        }
        else {
            turret_Rotation.setPower(rotation/3);
        }
    }

    public void TurretManualExtension (double turret_extension, double turret_retraction) {
        if (checker.checkIf(StateMachine.States.EXTENDED)) {
            if (((extendPosition > MAX_EXTENSION_TICKS && turret_extension > turret_retraction) || (extendPosition < MIN_EXTENSION_TICKS && turret_extension < turret_retraction))) {
                turret_Extension.setPower(0);
                op.telemetry.addData("extreme", extendPosition);

            }
            else if(abs(turret_extension-turret_retraction)<.2){
                turret_Extension.setPower((0));
                op.telemetry.addData("little", extendPosition);
            }
            else {
                turret_Extension.setPower((turret_extension - turret_retraction));
                op.telemetry.addData("tick", "ext", "retr", extendPosition, turret_extension, turret_retraction);

            }
        }
        op.telemetry.addData("turtext", extendPosition);

    }
    public void turretExtendo(double whereExtendo){
        if(whereExtendo>MAX_EXTENSION_TICKS){
            whereExtendo=MAX_EXTENSION_TICKS-5;
        }
        double distance = whereExtendo-extendPosition;
        if(abs(distance)<50){
            turret_Extension.setVelocity(0);
        }
        else {
            turret_Extension.setVelocity(distance/abs(distance)* 4 * (abs(distance) + 50));
        }
        if(abs(distance)<50&&turret_Extension.getVelocity()<100){
            turret_Extension.setVelocity(0);
            faked=true;
        }
        else{
            faked = false;
        }
        faked = true;
    }
    public void TurretManualFlip () {
        if (turret_Angle_Control.getPosition() > 0.5) {
            turret_Angle_Control.setPosition(0);
            turret_Angle_Control2.setPosition(0);
        }
        else {
            turret_Angle_Control.setPosition(1);
            turret_Angle_Control2.setPosition(1);
        }
    }

    public void FlipBasket (int up) {
        updateTurretPositions();

        if(checker.getState(StateMachine.States.BASKET_ARM_REST)&&areTeleop) {
            basketActuationServo.setPosition(0.68);
        }
        else{
            basketActuationServo.setPosition(0.18);
        }
    }
    public void FlipBasketToPosition (double torget) {
//        updateTurretPositions();
            basketActuationServo.setPosition(torget);
    }
    public void capBasket(){
//        if(!downCap) {
//            basketActuationServo.setPosition(0.0);
//            basketArmServo.setPosition(0.9);
//            AutoAngleControlRotating(0);
//            turret_Rotation.setTargetPosition((int) (0/DEG_PER_TICK_MOTOR));
//            turret_Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turret_Rotation.setPower(0.5);
//            downCap=true;
//        }
//        else{
//            FlipBasketArmToPosition(.45);
//            basketActuationServo.setPosition(0.07);
//            AutoAngleControlRotating(35);
//            turret_Extension.setTargetPosition(1500);
//            turret_Extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turret_Extension.setPower(1.0);
//            turret_Rotation.setTargetPosition((int) (-55/DEG_PER_TICK_MOTOR));
//            turret_Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turret_Rotation.setPower(0.5);
//            downCap=false;
//
//        }
    }
    public void FlipBasketArmToPosition (double torget) {
        updateTurretPositions();
        basketArmServo.setPosition(torget);
    }
    public void FlipBasketArmLow () {
        if(checker.getState(StateMachine.States.BASKET_ARM_REST)) {
            basketArmServo.setPosition(0.9);
        }
        else{
            basketArmServo.setPosition(0.00);
        }
        servoPos = !servoPos;
    }
    public void FlipBasketArmHigh () {
        updateTurretPositions();
        if(checker.getState(StateMachine.States.BASKET_ARM_REST)) {
            basketArmServo.setPosition(0.45);
        }
        else{
            basketArmServo.setPosition(0.01);
        }
    }

    public void SavePosition (int up) {
        double armLength = 10.5;
        if(up==1){
            armLength=10.5;
        }
        double outLength = extendPosition/TICKS_PER_INCH+armLength;

        turret_saved_positions[up][0][0] = turret_saved_positions[up][1][0];
        turret_saved_positions[up][0][1] = turret_saved_positions[up][1][1];
        turret_saved_positions[up][0][2] = turret_saved_positions[up][1][2];

        turret_saved_positions[up][1][2] = Math.sin(turret_Angle_Control.getPosition() * DEG_PER_TICK_SERVO * PI/180) * outLength;
        stopExtend();
        stopTurn();
        op.telemetry.addData("height", turret_saved_positions[up][1][2]);
        op.telemetry.addData("xdiff", (Math.sqrt(Math.pow(outLength, 2) - Math.pow(turret_saved_positions[up][1][2], 2))) * Math.sin(-(rotatePosition * DEG_PER_TICK_MOTOR + angle) * PI/180));
        double x = cos((-angle * Math.PI / 180));
        double y = sin((-angle * Math.PI / 180));
        double[] diff = {Math.sqrt(Math.pow(outLength, 2) - Math.pow(turret_saved_positions[up][1][2], 2)) * Math.sin(-(rotatePosition * DEG_PER_TICK_MOTOR + angle) * PI/180),Math.sqrt(Math.pow(outLength, 2) - Math.pow(turret_saved_positions[up][1][2], 2)) * Math.sin(-(rotatePosition * DEG_PER_TICK_MOTOR + angle) * PI/180)};
        turret_saved_positions[up][1][0] = xpos + (-y*diff[1]+x*diff[0]);
        turret_saved_positions[up][1][1] = ypos + (-x*diff[1]-y*diff[0]);
        op.telemetry.addData("x", turret_saved_positions[up][1][0]);
        op.telemetry.addData("y", turret_saved_positions[up][1][1]);
        op.telemetry.update();
    }

    public void UnsavePosition () {
//        turret_saved_positions[1][0] = turret_saved_positions[0][0];
//        turret_saved_positions[1][1] = turret_saved_positions[0][1];
//        turret_saved_positions[1][2] = turret_saved_positions[0][2];
    }

    public void TurretManualElevation (double elevation) { //stick_y
        double position;
        if (elevation < 0) {
            if (turret_Angle_Control.getPosition() >= 0.03) {
                position = turret_Angle_Control.getPosition() - 0.01;
                turret_Angle_Control.setPosition(position);
                turret_Angle_Control2.setPosition(position);
            }
        }
        else {
            if (turret_Angle_Control.getPosition() <= 0.97) {
                position = turret_Angle_Control.getPosition() + 0.01;
                turret_Angle_Control.setPosition(position);
                turret_Angle_Control2.setPosition(position);
            }
        }
    }

    public boolean TurretReset (double power) {
        boolean isReset = true;
        if(checker.getState(StateMachine.States.EXTENDED)){
            turret_Extension.setVelocity(-extendPosition/abs(extendPosition)* 4 * (abs(extendPosition) + 100));
        }
        else{
            turret_Extension.setVelocity(0);
        }

        if(checker.getState(StateMachine.States.TURRET_SHORT)&&!checker.getState(StateMachine.States.TURRET_STRAIGHT)){
            turret_Rotation.setVelocity(-rotatePosition/abs(rotatePosition)*(3*abs(rotatePosition)+100));
            FlipBasketToPosition(0.88);
        }
        else {
            turret_Rotation.setPower(0);
        }
        if(!arming){
            basketArmServo.setPosition(0.00);
            arming = true;
        }
        if(!basketing&&!checker.getState(StateMachine.States.TURRET_SHORT)){
            basketActuationServo.setPosition(.6);
            basketing = true;
        }
        if(!angleControlling&&checker.getState(StateMachine.States.TURRET_SHORT)) {
            turret_Angle_Control.setPosition(0);
            turret_Angle_Control2.setPosition(118.0/270);
            angleControlling = true;
        }
        op.telemetry.addData("basketArmRest", checker.getState(StateMachine.States.BASKET_ARM_REST));
        op.telemetry.addData("basketTransfer", checker.getState(StateMachine.States.BASKET_CIELING));
        op.telemetry.addData("Extended", checker.getState(StateMachine.States.EXTENDED));
        op.telemetry.addData("turretStraight", checker.getState(StateMachine.States.TURRET_STRAIGHT));
        if(checker.getState(StateMachine.States.BASKET_ARM_REST)&&checker.getState(StateMachine.States.BASKET_TRANSFER)&&!checker.getState(StateMachine.States.EXTENDED)&&checker.getState(StateMachine.States.TURRET_STRAIGHT)){
            isReset = false;
            arming = false;
            basketing = false;
            angleControlling = false;
            resetten=true;
        }
        else{
            resetten=false;
        }
        return isReset;
    }
    public void stopTurn(){
        turret_Rotation.setPower(0);
    }
    public void stopExtend(){
        turret_Extension.setPower(0);
    }
    public void TurretStop () {
        turret_Rotation.setPower(0);
        turret_Extension.setPower(0);
    }
}