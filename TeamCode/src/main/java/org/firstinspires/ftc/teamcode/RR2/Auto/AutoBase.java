package org.firstinspires.ftc.teamcode.RR2.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.RR2;

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
        robot.intializeIMU();
        robot.moveRobot(0.5,50);
        robot.slide.setPower(0);
        tensorFlowMineralDetection.runObjectDetection();
    }

    protected void dropDownFromLander() {
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

    protected void knockOffMineral(double leftRightAngle) {
        int currentDegree = 20;
        boolean detected = false;
        robot.uvcPivot.setPosition(currentDegree);
        objectDetection();
        while (detected = true || currentDegree >= 120) {
            if (tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT) {
                robot.finalTurn(-leftRightAngle);
                robot.finalMove(0.5, 58);
                detected = true;
            } else if (tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT) {
                robot.finalTurn(leftRightAngle);
                robot.finalMove(0.5, 58);
                detected = true;
            } else if (tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.CENTER || ) {
                robot.finalMove(0.5, 53);
                detected = true;
            } else {
                robot.uvcPivot.setPosition(currentDegree);
                currentDegree++;
            }
        }
        if(currentDregree >= 120) {
            robot.finalMove(0.5, 53);
        }
    }
}
