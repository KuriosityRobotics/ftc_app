package org.firstinspires.ftc.teamcode.RR2.Auto;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.RR2.RR2;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

public class AutoBase extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    public TensorFlowMineralDetection tensorFlowMineralDetection;

    public RR2 robot;
    public void initLogic(){
        //Init's robot
        tensorFlowMineralDetection = new TensorFlowMineralDetection(hardwareMap,telemetry,this);
        robot = new RR2(hardwareMap,telemetry,this);

        robot.fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tensorFlowMineralDetection.initVuforia();
        tensorFlowMineralDetection.initTfod();

        waitForStart();
        runtime.reset();
    }

    @Override
    public void runOpMode(){

    }

    protected void objectDetection(){
        telemetry.addLine(tensorFlowMineralDetection.runObjectDetection().toString());
        telemetry.update();
    }

    protected void dropDownFromLander() {
        objectDetection();
        robot.pivot.setPower(-1);
        robot.pivot2.setPower(1);

        while (robot.distance.getDistance(DistanceUnit.MM) > 150 && opModeIsActive()) {
            telemetry.addData("encoder value of Pivot", robot.distance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        telemetry.addData("done", "done");
        robot.pivot.setPower(0);
        robot.pivot2.setPower(0);

        robot.hangLockOpen();
        sleep(1000);

        robot.pivot.setPower(1);
        robot.pivot2.setPower(-1);

        while (robot.bottomDistance.getDistance(DistanceUnit.MM) > 23 && opModeIsActive()) {
            telemetry.addData("encoder value of Pivot", robot.pivot.getCurrentPosition());
            telemetry.update();
        }

        telemetry.update();
        robot.pivot.setPower(0);
        robot.pivot2.setPower(0);

        robot.hook.setPosition(0); //open
        sleep(1000);

        robot.pivot.setPower(-1);
        robot.pivot2.setPower(1);
        while (robot.distance.getDistance(DistanceUnit.MM) > 150 && opModeIsActive()) {
            telemetry.addData("encoder value of Pivot", robot.distance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        robot.pivot2.setPower(0);
        robot.pivot.setPower(0);
    }

    protected void navigateToDepotThenCrater() {
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.CENTER){
            robot.finalMove(0.5, -48);
        }else {
            robot.finalMove(0.5, -55);
        }

        //Getting to Depot
        robot.finalTurn(65);
        robot.goToWall(0.3,25);
        robot.finalTurn(135);
        robot.moveRobotKillSwitch(0.7,120,-120);
        robot.goToCrater(-0.7);
        telemetry.addData("Status","done");
        telemetry.update();
    }

    protected void knockOffMineral(double leftRightAngle) {
        robot.intializeIMU();
        robot.slide.setPower(0);
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
//            robot.finalTurn(-leftRightAngle);
//            robot.finalMove(0.5, 58);
            robot.cordinateMecanum(true, 4, 5, 0.5);
        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
//            robot.finalTurn(leftRightAngle);
//            robot.finalMove(0.5, 58);
            robot.cordinateMecanum(false, 4, 5, 0.5);
        } else {
            robot.finalMove(0.5, 53);
        }
    }

}
