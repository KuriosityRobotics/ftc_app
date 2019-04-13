package org.firstinspires.ftc.teamcode.Junior;

import android.content.Context;
import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;

import java.io.FileOutputStream;

import static android.content.Context.MODE_PRIVATE;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

public class Junior {

    //drive motors
    public DcMotor leftDrive;
    public DcMotor rightDrive;

    //imu
    private BNO055IMU imu;
    private Orientation angles;
    private Position position;

    //inherited classes from op mode
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private LinearOpMode linearOpMode;

    public Junior (HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        //config names need to match configs on the phone
        //Map drive motors
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");

        //Map LinearSlide Motors
        //Set direction of drive motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

    }

    public void intializeIMU() {
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

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
    }

    public void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void changeRunModeToUsingEncoder(){
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorMode(DcMotor.RunMode runMode){
        leftDrive.setMode(runMode);
        rightDrive.setMode(runMode);
    }

    public void timeMove() {
        long startTime = SystemClock.elapsedRealtime();

        leftDrive.setPower(1);
        rightDrive.setPower(1);

        linearOpMode.sleep(5000);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void driveMotorsBreakZeroBehavior() {
        //sets drive motors to brake mode
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void brakeRobot() {
        //brakes robot
        driveMotorsBreakZeroBehavior();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void turn (double targetHeading){
        targetHeading  = -1 * targetHeading;
        targetHeading = Range.clip(targetHeading, -179,179);
        double power;
        long startTime = SystemClock.elapsedRealtime();

        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;
        double maxAngle = Math.abs(startHeading - targetHeading);

        if (maxAngle < 1.0){
            return;
        }

        while(linearOpMode.opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            power = (targetHeading - angles.firstAngle) / maxAngle;
            if(Math.abs(power) < 0.01){
                if (power > 0){
                    power = 0.01;
                } else if (power < 0){
                    power = -0.01;
                } else {
                    break;
                }
            }
            if (Math.abs(angles.firstAngle - startHeading) > maxAngle || ((SystemClock.elapsedRealtime() - startTime) > 2000)){
                break;
            }
            leftDrive.setPower(-power);
            rightDrive.setPower(power);
        }
        brakeRobot();
        linearOpMode.sleep(100);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void splineMove(double[][] data, double maxSpeed) {

        resetEncoders();
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        long startTime = SystemClock.elapsedRealtime();
        double refLeftSpeed, refRightSpeed;
        double refLeftDistance, refRightDistance;
        double refHeading;
        double leftPower, rightPower;
        double leftDistance, rightDistance;
        double heading;
        int inc;
        int i;
        double encoderToInches = 60/5150;  //5150 encoders = 60 inches
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;

        //create file

        /*double maxSpeed = 0;
        for (i=0; i<data.length; i++){
            if (maxSpeed < data[i][0]){
                maxSpeed = data[i][0];
            }
            if (maxSpeed < data[i][1]){
                maxSpeed = data[i][1];
            }
        }
        */

        while (true){

            double dt = SystemClock.elapsedRealtime() - startTime; //in milli
            dt = dt / 1000; //in seconds

            if (dt < data[data.length - 1][2]){                //find increment at any time to do interpolation
                inc = -1;
                for (i=0; i<data.length - 2; i++){
                    if (data[i][2] <= dt && dt < data[i+1][2]){
                        inc = i;
                        break;
                    }
                }
                if (inc < 0){
                    brakeRobot();
                    break;
                }
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                // find the left and right speed by interpolation from data file
                refLeftSpeed  = ((data[inc+1][0] - data[inc][0]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][0];
                refRightSpeed = ((data[inc+1][1] - data[inc][1]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][1];

                // find the left and right distance by interpolation from data file
                refLeftDistance = ((data[inc+1][4] - data[inc][4]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][5];
                refRightDistance = ((data[inc+1][5] - data[inc][5]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][5];

                // find the heading by interpolation from data file
                refHeading =((data[inc+1][6] - data[inc][6]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][6] + startHeading;

                // find the left and right encoder values and convert them to distance traveled in inches
                leftDistance = leftDrive.getCurrentPosition() * encoderToInches;
                rightDistance = rightDrive.getCurrentPosition() * encoderToInches;

                // find the heading of robot
                heading = angles.firstAngle;

                // old algorithm
                //leftPower = refLeftSpeed/maxSpeed;
                //rightPower = refRightSpeed/maxSpeed;

                // find power
                leftPower =  refLeftSpeed /maxSpeed + (refLeftDistance  - leftDistance ) / 10000 + (refHeading - heading) / 100;
                rightPower = refRightSpeed/maxSpeed + (refRightDistance - rightDistance) / 10000 + (refHeading - heading) / 100;

                // set power
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);

                //write to file
                telemetry.addLine(dt+ " - LeftDistance:" + (refLeftDistance-leftDistance) + " RightDistance:" + (refRightDistance-rightDistance) + " leftPower:" + (refLeftSpeed-leftPower) + " rightPower:" + (refRightSpeed-rightPower) + " heading:" + (refHeading - heading) );
                telemetry.update();
                
            } else {
                brakeRobot();
                break;
            }
        }

    }
}
