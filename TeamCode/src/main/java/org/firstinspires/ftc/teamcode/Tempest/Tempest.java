package org.firstinspires.ftc.teamcode.Tempest;

/**
 * Created by sam on 1/21/18.
 */

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Kuro on 10/29/2017.
 */

public class Tempest {
    //Drive Motors
    public DcMotor FLeft;
    public DcMotor FRight;
    public DcMotor BLeft;
    public DcMotor BRight;

    //Ints
    public double speedSet;

    //Intake Motors;
    public DcMotor lSlideLeft;
    public DcMotor lSlideRight;
    public DcMotor lSlideUp;

    //Intake Motors & Servos
    public DcMotor intake;
    public Servo iRight;
    public Servo iLeft;
    //imu
    public BNO055IMU imu;
    public Orientation angles;
    public Position position;

    //Inherited classes from Op Mode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode linearOpMode;

    static double linearSlideValue = 0.7;

    public Tempest(HardwareMap hardwareMap, Telemetry telemetry,LinearOpMode linearOpMode){

        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        //Map drive motors
        FLeft = hardwareMap.dcMotor.get("fLeft");
        FRight = hardwareMap.dcMotor.get("fRight");
        BLeft = hardwareMap.dcMotor.get("bLeft");
        BRight = hardwareMap.dcMotor.get("bRight");

        //Map LinearSlide Motors


        //Set direction of drive motors
        FLeft.setDirection(DcMotor.Direction.FORWARD);
        FRight.setDirection(DcMotor.Direction.REVERSE);
        BLeft.setDirection(DcMotor.Direction.FORWARD);
        BRight.setDirection(DcMotor.Direction.REVERSE);
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
        FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void resumeEncoders(){
        FLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void changeRunModeToUsingEncoder(){
        FLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void setMotorMode(DcMotor.RunMode runMode){
        FLeft.setMode(runMode);
        FRight.setMode(runMode);
        BLeft.setMode(runMode);
        BRight.setMode(runMode);
    }



    public void setDrivePower(double power){
        FLeft.setPower(power);
        FRight.setPower(power);
        BLeft.setPower(power);
        BRight.setPower(power);
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
            FLeft.setPower(-power);
            FRight.setPower(power);
            FLeft.setPower(-power);
            BRight.setPower(power);
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
        lSlideLeft.setPower(linearSlideValue);
        lSlideRight.setPower(linearSlideValue);
    }
    public void  moveLinearSlideDown() {
        lSlideLeft.setPower(-linearSlideValue);
        lSlideRight.setPower(-linearSlideValue);
    }
    public void allWheelDrive(double fLeftPower, double bLeftPower, double fRightPower, double bRightPower) {
        FLeft.setPower(fLeftPower);
        FRight.setPower(fRightPower);
        BRight.setPower(bRightPower);
        BLeft.setPower(bLeftPower);
    }
    public void fLeft(double power) {
        FLeft.setPower(power * speedSet);
    }
    public void fRight(double power) {
        FLeft.setPower(power * speedSet);
    }
    public void bLeft(double power) {
        BLeft.setPower(power * speedSet);
    }
    public void bRight(double power) {
        BRight.setPower(power * speedSet);
    }
    public void OutTake() {
        iRight.setPosition(1);
        iLeft.setPosition(1);
        intake.setPower(-1);
    }
    public void InTake() {
        iRight.setPosition(-1);
        iLeft.setPosition(-1);
        intake.setPower(1);
    }
    public void moveRobot(double speed, int targetPostition, long timeInMilli){

        long startTime = SystemClock.elapsedRealtime();

        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        double newSpeed = speed;

        if(targetPostition<0){
            newSpeed = newSpeed * -1;
        }

        FLeft.setPower(newSpeed);
        FRight.setPower(newSpeed);
        BLeft.setPower(newSpeed);
        BRight.setPower(newSpeed);


        FLeft.setTargetPosition(targetPostition);
        FRight.setTargetPosition(targetPostition);
        BLeft.setTargetPosition(targetPostition);
        BRight.setTargetPosition(targetPostition);


        while(FLeft.isBusy() && FRight.isBusy() && BLeft.isBusy() && BRight.isBusy()
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

        FLeft.setPower(speed);
        FRight.setPower(speed);
        BLeft.setPower(speed);
        BRight.setPower(speed);

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
            FLeft.setPower(-power);
            FRight.setPower(power);
            BLeft.setPower(-power);
            BRight.setPower(power);
        }

        setDrivePower(0);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}