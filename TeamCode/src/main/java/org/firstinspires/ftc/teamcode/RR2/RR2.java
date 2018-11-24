package org.firstinspires.ftc.teamcode.RR2;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

public class RR2 {
    //Drive Motors
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;

    private RevTouchSensor upTouch;

    public Rev2mDistanceSensor distance;

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
    private BNO055IMU imu;
    private Orientation angles;
    private Position position;

    //Inherited classes from Op Mode
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private LinearOpMode linearOpMode;


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

        distance = hardwareMap.get(Rev2mDistanceSensor.class,"distance");


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
        brakeRobot();
        linearOpMode.sleep(100);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void finalMove(double speed, double targetDistance) {
        changeRunModeToUsingEncoder();
        resetEncoders();
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double rotations = targetDistance * (400/17.5);

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

            if(scaleFactor<0.15){
                scaleFactor = 0.15;
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

    public void moveRobot(double speed, int targetPostition) {
        moveRobot(speed, targetPostition, 10000);
        double currentPosition;
        brakeRobot();
    }

    public void moveRobot(double speed, int targetPostition, long timeInMilli){

        long startTime = SystemClock.elapsedRealtime();

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
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void brakeRobot() {
        driveMotorsBreakZeroBehavior();
        fLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);
        bLeft.setPower(0);
    }
}