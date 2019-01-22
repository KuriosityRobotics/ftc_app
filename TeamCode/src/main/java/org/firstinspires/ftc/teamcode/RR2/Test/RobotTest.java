package org.firstinspires.ftc.teamcode.RR2.Test;

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

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

public class RobotTest {
    //Drive Motors
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;

    public Rev2mDistanceSensor distance;
    public Rev2mDistanceSensor bottomDistance;
    public Rev2mDistanceSensor frontRightDistance;
    public Rev2mDistanceSensor frontDistance;
    public Rev2mDistanceSensor frontFacingLeft;
    public Rev2mDistanceSensor hangDistance;


    //Intake Motors;
    public DcMotor slide;
    public DcMotor pivot;
    public DcMotor pivot2;

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


    public RobotTest(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode){

        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        //config names need to match configs on the phone
        //Map drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        intake = hardwareMap.dcMotor.get("intake");
        blocker = hardwareMap.servo.get("blocker");
//
        slide = hardwareMap.dcMotor.get("slide");
        pivot = hardwareMap.dcMotor.get("pivot");
        pivot2 = hardwareMap.dcMotor.get("pivot2");
        hangLockLeft = hardwareMap.servo.get("hangLockLeft");
        hangLockRight = hardwareMap.servo.get("hangLockRight");
        hook = hardwareMap.servo.get("hook");
//
        distance = hardwareMap.get(Rev2mDistanceSensor.class,"distance");
        bottomDistance = hardwareMap.get(Rev2mDistanceSensor.class,"bottomDistance");
//        frontRightDistance = hardwareMap.get(Rev2mDistanceSensor.class,"frontRightDistance");
        frontDistance = hardwareMap.get(Rev2mDistanceSensor.class,"frontFacingRight");
//        frontFacingLeft = hardwareMap.get(Rev2mDistanceSensor.class,"frontFacingLeft");
//        hangDistance = hardwareMap.get(Rev2mDistanceSensor.class,"hangDistance");

        //Map LinearSlide Motors
        //Set direction of drive motors
        fLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);
    }
}