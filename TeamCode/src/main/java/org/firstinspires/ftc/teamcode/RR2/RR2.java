package org.firstinspires.ftc.teamcode.RR2;

/**
 * Created by sam on 1/21/18.
 */

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Kuro on 10/29/2017.
 */

public class RR2 {
    //Drive Motors
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;

    RevTouchSensor upTouch;

    //Ints
    public double speedSet;

    //Intake Motors;
    public DcMotor slide;
    public DcMotor pivot;

    //Intake Motors & Servos
    public DcMotor intake;
    public Servo blocker;
    public Servo hangLockLeft;
    public Servo hangLockRight;
    public Servo hook;
    //imu
    public BNO055IMU imu;
    public Orientation angles;
    public Position position;

    //Inherited classes from Op Mode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode linearOpMode;

    boolean clawsOnOff = true;
    static double linearSlideValue = 0.7;

    public RR2(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode){

        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        //Map drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        intake = hardwareMap.dcMotor.get("intake");
        blocker = hardwareMap.servo.get("blocker");

        slide = hardwareMap.dcMotor.get("slide");
        pivot = hardwareMap.dcMotor.get("pivot");
        hangLockLeft = hardwareMap.servo.get("hangLockLeft");
        hangLockRight = hardwareMap.servo.get("hangLockRight");
        hook = hardwareMap.servo.get("hook");

        upTouch = hardwareMap.get(RevTouchSensor.class,"upTouch");


        //Map LinearSlide Motors
        //Set direction of drive motors
        fLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);
    }



    public void intializeIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    public void resetEncoders(){
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void resumeEncoders(){
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void changeRunModeToUsingEncoder(){
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void setMotorMode(DcMotor.RunMode runMode){
        fLeft.setMode(runMode);
        fRight.setMode(runMode);
        bLeft.setMode(runMode);
        bRight.setMode(runMode);
    }



    public void setDrivePower(double power){
        fLeft.setPower(power);
        fRight.setPower(power);
        bLeft.setPower(power);
        bRight.setPower(power);
    }



    public void finalTurn(double targetHeading){
        finalTurn(targetHeading, 10000);
    }

    public void finalTurn(double targetHeading,long timeInMilli){
        targetHeading = Range.clip(targetHeading, -179, 179);

        long startTime = SystemClock.elapsedRealtime();

        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        position = imu.getPosition();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;
        double maxAngle = startHeading - targetHeading;
        maxAngle = Math.abs(maxAngle);

        int sign = 0;
        if(targetHeading > startHeading){
            sign = 1;
        }else{
            sign = -1;
        }
        if(maxAngle == 0){
            return;
        }
        while(linearOpMode.opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double currentDeltatAngle = Math.abs(angles.firstAngle - startHeading);
            double scaleFactor = currentDeltatAngle / maxAngle;
            double absolutePower = 1-scaleFactor;

            if(absolutePower< 0.01){
                absolutePower = 0.01;
            }
            double power = absolutePower * sign;
            if(scaleFactor > 1 || ((SystemClock.elapsedRealtime() - startTime) > timeInMilli)){
                break;
            }
            fLeft.setPower(-power);
            fRight.setPower(power);
            fLeft.setPower(-power);
            bRight.setPower(power);
        }

        setDrivePower(0);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void moveRobotInches(double speed, double targetDistance){
        moveRobot(speed, (int)(targetDistance / 22.25 * 1000));
        brakeMotors();
    }
    public void moveRobot(double speed, int targetPostition) {
        moveRobot(speed, targetPostition, 10000);
        brakeMotors();
    }

    public void  moveLinearSlideUp() {
        slide.setPower(linearSlideValue);
    }
    public void  moveLinearSlideDown() {
        slide.setPower(-linearSlideValue);
    }
    public void allWheelDrive(double fLeftPower, double bLeftPower, double fRightPower, double bRightPower) {
        fLeft.setPower(fLeftPower);
        fRight.setPower(fRightPower);
        bRight.setPower(bRightPower);
        bLeft.setPower(bLeftPower);
    }
    public void fLeft(double power) {
        fLeft.setPower(power * speedSet);
    }
    public void fRight(double power) {
        fLeft.setPower(power * speedSet);
    }
    public void bLeft(double power) {
        bLeft.setPower(power * speedSet);
    }
    public void bRight(double power) {
        bRight.setPower(power * speedSet);
    }

    public void moveRobot(double speed, int targetPostition, long timeInMilli){

        long startTime = SystemClock.elapsedRealtime();

        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        double newSpeed = speed;

        if(targetPostition<0){
            newSpeed = newSpeed * -1;
        }

        fLeft.setPower(newSpeed);
        fRight.setPower(newSpeed);
        bLeft.setPower(newSpeed);
        bRight.setPower(newSpeed);


        fLeft.setTargetPosition(targetPostition);
        fRight.setTargetPosition(targetPostition);
        bLeft.setTargetPosition(targetPostition);
        bRight.setTargetPosition(targetPostition);


        while(fLeft.isBusy() && fRight.isBusy() && bLeft.isBusy() && bRight.isBusy()
                && (SystemClock.elapsedRealtime() - startTime < timeInMilli) && linearOpMode.opModeIsActive()){
        }

        brakeMotors();

        telemetry.addLine("finished sleeping");
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public String getColor(ColorSensor colorSensor){
        if(colorSensor.blue() > colorSensor.red()){
            return "red";
        }else{
            return "blue";
        }
    }

    public void brakeMotors(){
        setDrivePower(0);
    }



    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void autoNav(double speed, double x_start, double y_start, double x_end, double y_end){
        double distanceToTravel = 0;
        distanceToTravel = Math.sqrt(((x_start-x_end)*(x_start-x_end))+((y_start-y_end)*(y_start-y_end)));
        double oppositeSide = Math.abs(y_end-y_start);
        double lengthB = distanceToTravel;
        double adjacentSjde = Math.abs(x_end-x_start);
        double angle = 0;
        angle = Math.atan(oppositeSide/adjacentSjde);
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("angle",angle);
        telemetry.addData("distance",distanceToTravel);
        telemetry.update();
        finalTurn(angle);

        fLeft.setPower(speed);
        fRight.setPower(speed);
        bLeft.setPower(speed);
        bRight.setPower(speed);

        moveRobotInches(speed,distanceToTravel*12);

    }

    public void multiplAutofinalTurn(double targetHeading,long timeInMilli) {
        targetHeading = Range.clip(targetHeading, -179, 179);

        long startTime = SystemClock.elapsedRealtime();

        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        position = imu.getPosition();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;
        double maxAngle = startHeading - targetHeading;
        maxAngle = Math.abs(maxAngle);

        int sign = 0;
        if (targetHeading > startHeading) {
            sign = 1;
        } else {
            sign = -1;
        }
        if (maxAngle == 0) {
            return;
        }

        while (linearOpMode.opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double currentDeltatAngle = Math.abs(angles.firstAngle - startHeading);
            double scaleFactor = currentDeltatAngle / maxAngle;
            double absolutePower = 1 - scaleFactor;

            if (absolutePower < 0.01) {
                absolutePower = 0.01;
            }
            double power = absolutePower * sign;
            if (scaleFactor > 1 || ((SystemClock.elapsedRealtime() - startTime) > timeInMilli)) {
                break;
            }
            fLeft.setPower(-power);
            fRight.setPower(power);
            bLeft.setPower(-power);
            bRight.setPower(power);
        }
        setDrivePower(0);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}