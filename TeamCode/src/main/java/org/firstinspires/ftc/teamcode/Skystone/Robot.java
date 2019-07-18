package org.firstinspires.ftc.teamcode.Skystone;


import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import java.util.Timer;

import java.util.Timer;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

public class Robot {
    //Drive Motors
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;

    //imu
    private BNO055IMU imu;
    private Orientation angles;
    private Position position;

    //Inherited classes from Op Mode
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private LinearOpMode linearOpMode;

    //for odometry
    double xPosGlobal = 0;
    double yPosGlobal = 0;
    double angleGlobal = 0;

    double fLeftOLD;
    double fRightOLD;
    double bLeftOLD;
    double bRightOLD;

    //PID (concept only)



    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        //config names need to match configs on the phone
        //Map drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        //Set direction of drive motors
        fLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setDrivePower(double power){
        fLeft.setPower(power);
        fRight.setPower(power);
        bLeft.setPower(power);
        bRight.setPower(power);
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

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
    }

    public void resetEncoders(){
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    //normal use method default 10 second kill time
    public void finalTurn(double targetHeading){
        finalTurn(targetHeading, 2000);
    }

    public void coordinateMecanum(int xCord, int yCord, double speed) {
        intializeIMU();
        double degrees = 1.5 * Math.PI - Math.atan2(yCord,xCord);
        position = imu.getPosition();
        int targetLength = (int)(Math.sqrt(Math.pow(2, xCord) + Math.pow(2, yCord))/ 0.028);
        while (linearOpMode.opModeIsActive()) {
            fLeft.setPower((Math.sqrt(2) * Math.cos(1 / 57.29 * (degrees - 45))) * speed);
            bRight.setPower((Math.sqrt(2) * Math.cos(1 / 57.29 * (degrees - 45))) * speed);
            fRight.setPower((Math.sqrt(2) * Math.cos(1 / 57.29 * degrees)) * 0.5);
            bLeft.setPower((Math.sqrt(2) * Math.cos(1 / 57.29 * degrees)) * 0.5);
        }

        if (xCord > 0 && yCord > 0) {
            targetLength = (int)(Math.sqrt(Math.pow(2, xCord) + Math.pow(2, yCord))/ 0.028);
            fLeft.setTargetPosition(targetLength);
            bRight.setTargetPosition(targetLength);
            while(fLeft.isBusy()) {
                fLeft.setPower(speed);
                bRight.setPower(speed);
                if (speed < 0) {
                    fRight.setPower((yCord - xCord) * (-0.01 * speed));
                    bLeft.setPower((yCord - xCord) * (-0.01 * speed));
                } else {
                    fRight.setPower(yCord - xCord * (0.01 * speed));
                    bLeft.setPower(yCord - xCord* (0.01 * speed));
                }
            }

        }
        else {
            targetLength = (int)Math.sqrt(Math.pow(2, xCord) + Math.pow(2, yCord));
            fRight.setTargetPosition(targetLength);
            bLeft.setTargetPosition(targetLength);
            while(fLeft.isBusy()) {
                fRight.setPower(speed);
                bLeft.setPower(speed);
                if (speed < 0) {
                    fLeft.setPower((yCord - xCord) * (-0.01 * speed));
                    bRight.setPower((yCord - xCord) * (-0.01 * speed));
                } else {
                    fLeft.setPower(yCord - xCord* (0.01 * speed));
                    bRight.setPower(yCord - xCord * (0.01 * speed));
                }
            }
        }
    }
    //turn method with timed kill switch
    public void finalTurn(double targetHeading, long timeInMilli){
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
            bLeft.setPower(-power);
            bRight.setPower(power);
        }
        brakeRobot();
        linearOpMode.sleep(100);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void allWheelDrive(double fLpower,double fRpower, double bLpower, double bRpower) {
        fLeft.setPower(fLpower);
        fRight.setPower(fRpower);
        bLeft.setPower(bLpower);
        bRight.setPower(bRpower);
    }

    public void resetMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void finalMove(double speed, double targetDistance) {
        //move robot function
        //to move backwards make targetDistance negative
        double rotations = 0;
        if (targetDistance>0) {
            rotations = targetDistance / 0.0168;
        } else {
            rotations = targetDistance / 0.0156;
        }
        moveRobot(speed,(int)(rotations));

        brakeRobot();
        linearOpMode.sleep(100);
    }

    public void testMoveStraight(double speed, double targetDistance){
        //experimental don't use
        //corrects the robot to move straight
        //to move backwards make targetDistance negative
        double rotations = 0;
        if (targetDistance>0) {
            rotations = targetDistance / 0.028;
        } else {
            rotations = targetDistance / 0.026;
        }
        moveRobot(speed,(int)(rotations));
        changeRunModeToUsingEncoder();
        resetEncoders();
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotations = targetDistance * (400/17.5);

        double leftPower = speed;
        double rightPower = speed;

        double straightCorrection;

        boolean isNegative = false;

        double deccelerationLeft;
        double deccelerationRight;

        if (targetDistance < 0) {
            isNegative = true;
        }

        if (!isNegative) {
            rotations = targetDistance / 0.028;
        } else {
            rotations = targetDistance / 0.026;
        }

        fLeft.setTargetPosition((int)rotations);
        bLeft.setTargetPosition((int)rotations);
        fRight.setTargetPosition((int)rotations);
        bRight.setTargetPosition((int)rotations);

        fLeft.setPower(speed);
        fRight.setPower(speed);
        bLeft.setPower(speed);
        bRight.setPower(speed);

        double scaleFactor;
        double currentPositon;

        double difference = 0;
        while((fLeft.isBusy() && fRight.isBusy()) && linearOpMode.opModeIsActive()){

            currentPositon = Math.abs(bLeft.getCurrentPosition());

            scaleFactor = Math.abs(Math.abs(rotations - currentPositon) / rotations);

            difference = bLeft.getCurrentPosition() - bRight.getCurrentPosition();
            straightCorrection = Math.abs(difference) / 1000;

            if(scaleFactor<0.4){
                scaleFactor = 0.4;
            }

            deccelerationLeft = leftPower;
            deccelerationRight = rightPower;

            if (!isNegative) {
                if (difference < 0) {
                    deccelerationRight -= straightCorrection;
                }
                if (difference > 0) {
                    deccelerationLeft += straightCorrection;
                }
            } else {
                if (difference > 0) {
                    deccelerationLeft -= straightCorrection;
                }
                if (difference < 0) {
                    deccelerationRight -= straightCorrection;
                }
            }

            deccelerationLeft = deccelerationLeft * scaleFactor;
            deccelerationRight = deccelerationRight * scaleFactor;

            telemetry.addLine("LeftPower: " + deccelerationLeft);
            telemetry.addLine("RightPower: " + deccelerationRight);
            telemetry.addLine("correction: " + straightCorrection);
            telemetry.addLine("Difference: " + difference);

            telemetry.update();
            fLeft.setPower(deccelerationLeft);
            fRight.setPower(deccelerationRight);
            bLeft.setPower(deccelerationLeft);
            bRight.setPower(deccelerationRight);

            telemetry.addLine("Target position: " + rotations);

            telemetry.addLine("Right position: " + fRight.getCurrentPosition());
            telemetry.addLine("Left position: " + fLeft.getCurrentPosition());
            telemetry.update();
        }
        brakeRobot();
        linearOpMode.sleep(100);
    }

    public void moveRobot(double speed, int targetPostition){
        //called by final move - bare bones move function
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        double newSpeed = speed;

        if(targetPostition<0){
            newSpeed = newSpeed * -1;
            fLeft.setPower(newSpeed);
            fRight.setPower(newSpeed);
            bLeft.setPower(newSpeed);
            bRight.setPower(newSpeed);
        }else{
            fLeft.setPower(newSpeed);
            fRight.setPower(newSpeed);
            bLeft.setPower(newSpeed);
            bRight.setPower(newSpeed);
        }

        fLeft.setTargetPosition(targetPostition);
        fRight.setTargetPosition(targetPostition);
        bLeft.setTargetPosition(targetPostition);
        bRight.setTargetPosition(targetPostition);

        while(fLeft.isBusy() && fRight.isBusy() && bLeft.isBusy() && bRight.isBusy()
                && linearOpMode.opModeIsActive()){
        }
        brakeRobot();

        telemetry.addLine("finished sleeping");
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveMotorsBreakZeroBehavior() {
        //sets drive motors to brake mode
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void brakeRobot() {
        //brakes robot
        driveMotorsBreakZeroBehavior();
        fLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);
        bLeft.setPower(0);
    }

    public void wallFollow(double speed, double distance){
        //experimental - don't use
        //follows wall
        boolean notTimeLimit = true;
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLeft.setPower(speed);
        fRight.setPower(speed);
        bLeft.setPower(speed);
        bRight.setPower(speed);

        fLeft.setTargetPosition((int)(distance/0.028));
        fRight.setTargetPosition((int)(distance/0.028));
        bLeft.setTargetPosition((int)(distance/0.028));
        bRight.setTargetPosition((int)(distance/0.028));

        while(fLeft.isBusy() && linearOpMode.opModeIsActive()){
//            double leftPower = speed+normalizedDelta(frontRightDistance.getDistance(MM),70)/1000;
//            double rightPower = speed+normalizedDelta(backRightDistance.getDistance(MM),70)/1000;
//
//            fLeft.setPower(leftPower);
//            bLeft.setPower(leftPower);
//            fRight.setPower(rightPower);
//            bRight.setPower(rightPower);
//
//            telemetry.addData("frontRightDistance",frontRightDistance.getDistance(MM));
//            telemetry.addData("backRightDistance",backRightDistance.getDistance(MM));
//            telemetry.update();
//            if(frontDistance.getDistance(MM)<200 || frontFacingLeft.getDistance(MM)<200){
//                fLeft.setPower(0);
//                fRight.setPower(0);
//                bLeft.setPower(0);
//                bRight.setPower(0);
//                long startTime = SystemClock.elapsedRealtime();
//                while (((SystemClock.elapsedRealtime() - startTime) < 3000)){
//                    if(frontDistance.getDistance(MM)>200 && frontFacingLeft.getDistance(MM)>200){
//                        notTimeLimit = false;
//                        break;
//                    }else{
//                        notTimeLimit = true;
//                    }
//                }
//                if(notTimeLimit){
//                    finalTurn(0);
//                    wallFollow(0.7,-fLeft.getCurrentPosition()*0.028);
//                    brakeRobot();
//                }
//            }
        }
        brakeRobot();
    }

    private double normalizedDelta(double distance, double constant){
        //called in wallFollow
        double delta = distance-constant;
        if(delta>-50 && delta<50){
            delta = 0;
        }
        return delta;
    }
    public void goToCrater(double speed){
        //goes to crater using the imu and detects a change in elevation
        position = imu.getPosition();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPosition = angles.thirdAngle;
        telemetry.addLine(Double.toString(currentPosition));
        telemetry.update();
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.setPower(speed);
        fRight.setPower(speed);
        bLeft.setPower(speed);
        bRight.setPower(speed);

        while(linearOpMode.opModeIsActive() && Math.abs(angles.thirdAngle - currentPosition)<1 ){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("X",angles.thirdAngle);
            telemetry.addData("Y",angles.secondAngle);
            telemetry.addData("Z",angles.firstAngle);
            telemetry.update();
        }
        brakeRobot();
    }


    public void setBrakeModeDriveMotors(){
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void polarMovement(double speed, double radiusInches, double degrees){

        setBrakeModeDriveMotors();

        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        double radiusEncoders = radiusInches * 119.05;
        double radians = Math.toRadians(degrees);
        double y = radiusEncoders/Math.sin(radians);
        double x = radiusEncoders/Math.cos(radians);

        double fL = y - x;
        double fR = y + x;

        fRight.setPower(speed);
        fLeft.setPower(speed);
        bLeft.setPower(speed);
        bRight.setPower(speed);

        fRight.setTargetPosition((int)(fR * radiusEncoders));
        fLeft.setTargetPosition((int)(fL * radiusEncoders));
        bLeft.setTargetPosition((int)(fR * radiusEncoders));
        bRight.setTargetPosition((int)(fL * radiusEncoders));

//        fRight.setPower(0);
//        fLeft.setPower(1);
//        bLeft.setPower(0);
//        bRight.setPower(1);
//
//        fRight.setTargetPosition(0);
//        fLeft.setTargetPosition(1190);
//        bLeft.setTargetPosition(0);
//        bRight.setTargetPosition(1190);

        while(linearOpMode.opModeIsActive() && ((fLeft.isBusy() && bRight.isBusy()) || (fRight.isBusy() && bLeft.isBusy()))){
            telemetry.addLine("fRight: " + Integer.toString(fRight.getCurrentPosition()));
            telemetry.addLine("bLeft: " + Integer.toString(bLeft.getCurrentPosition()));
            telemetry.update();
        }
        brakeRobot();
    }

    public void timeMove (double[][] data){


        long startTime = SystemClock.elapsedRealtime();

        fRight.setPower(1);
        fLeft.setPower(1);
        bLeft.setPower(1);
        bRight.setPower(1);

        while(SystemClock.elapsedRealtime() - startTime < 5000){
            linearOpMode.sleep(1);
        }

        fRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);

    }

    public void splineMove(double[][] data) {

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
        double encoderToInches = 0.156;  //515 encoders = 8 inches
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;

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


        while (linearOpMode.opModeIsActive()){

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
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                // find the left and right speed by interpolation from data file
                refLeftSpeed  = ((data[inc+1][0] - data[inc][0]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][0];
                refRightSpeed = ((data[inc+1][1] - data[inc][1]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][1];

                // find the left and right distance by interpolation from data file
                refLeftDistance = ((data[inc+1][4] - data[inc][4]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][5];
                refRightDistance = ((data[inc+1][5] - data[inc][5]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][5];

                // find the heading by interpolation from data file
                refHeading =((data[inc+1][6] - data[inc][6]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][6] + startHeading;

                // find the left and right encoder values and convert them to distance traveled in inches
                leftDistance = fLeft.getCurrentPosition() * encoderToInches;
                rightDistance = fRight.getCurrentPosition() * encoderToInches;

                // find the heading of robot
                heading = angles.firstAngle;

                // find power
                leftPower  = refLeftSpeed;
                rightPower = refRightSpeed;

                // set power
                fLeft.setPower(leftPower);
                bLeft.setPower(leftPower);
                fRight.setPower(rightPower);
                bRight.setPower(rightPower);
            } else {
                brakeRobot();
                break;
            }
        }

    }

    public double[] getPos() {
        double[] positions = new double[3];
        final double radius = 2;
        final double encoderPerRevolution = 537.6;
        final double l = 7;
        final double b = 6.5;

        double fl = 2 * Math.PI * fLeft.getCurrentPosition() / encoderPerRevolution; //radians each motor has travelled
        double fr = 2 * Math.PI * fRight.getCurrentPosition() / encoderPerRevolution;
        double bl = 2 * Math.PI * bLeft.getCurrentPosition() / encoderPerRevolution;
        double br = 2 * Math.PI * bRight.getCurrentPosition() / encoderPerRevolution;

        double xPos = radius/4 * (fl + bl + br + fr);
        double yPos = radius/4 * (-fl + bl - br + fr);
        double angle = radius/4 *(-fl/(l+b) - bl/(l+b) + br/(l+b) + fr/(l+b));
        angle = angle * 180/(2*Math.PI);

        positions[0] = xPos;
        positions[1] = yPos;
        positions[2] = angle;
        return positions;
    }

    public double[] odometry() {

        final double radius = 2;
        final double encoderPerRevolution = 806.4;
        final double l = 7;
        final double w = 6.5;

        // find robot position
        double fl = 2 * Math.PI * fLeft.getCurrentPosition() / encoderPerRevolution; //radians each motor has travelled
        double fr = 2 * Math.PI * fRight.getCurrentPosition() / encoderPerRevolution;
        double bl = 2 * Math.PI * bLeft.getCurrentPosition() / encoderPerRevolution;
        double br = 2 * Math.PI * bRight.getCurrentPosition() / encoderPerRevolution;

        double xPosRobot = radius/4 * (fl + bl + br + fr);
        double yPosRobot = radius/4 * (-fl + bl - br + fr);
        double angleRobot = radius/4 *(-fl/(l+w) - bl/(l+w) + br/(l+w) + fr/(l+w));

        //converting to global frame
        xPosGlobal += (Math.cos(angleGlobal) * Math.sin(angleRobot) - (Math.cos(angleRobot) - 1) * Math.sin(angleGlobal)) * xPosRobot / angleRobot + (Math.cos(angleGlobal) * (Math.cos(angleRobot) - 1) - Math.sin(angleGlobal) * Math.sin(angleRobot)) * yPosRobot / angleRobot;
        yPosGlobal += ((Math.cos(angleRobot) - 1) * Math.sin(angleGlobal) + (Math.cos(angleGlobal)) * Math.sin(angleRobot)) * yPosRobot / angleRobot + (Math.cos(angleGlobal) * (Math.cos(angleRobot) - 1) + Math.sin(angleGlobal) * Math.sin(angleRobot)) * xPosRobot / angleRobot;
        angleGlobal += angleRobot;

        fLeftOLD = fLeft.getCurrentPosition();
        fRightOLD = fRight.getCurrentPosition();
        bLeftOLD = bLeft.getCurrentPosition();
        bRightOLD = bRight.getCurrentPosition();

        //inserting into return array
        return new double[] { xPosGlobal, yPosGlobal, angleGlobal };
    }

    public double getxPosGlobal() {
        return xPosGlobal;
    }

    public double getyPosGlobal() {
        return yPosGlobal;
    }

    public double getAngleGlobal() {
        return angleGlobal;
    }


    public void getData(){

        fLeft.setPower(1);
        fRight.setPower(1);
        bLeft.setPower(1);
        bRight.setPower(1);

        linearOpMode.sleep(1000);

    }

}