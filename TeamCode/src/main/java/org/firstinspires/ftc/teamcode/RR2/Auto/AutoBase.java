package org.firstinspires.ftc.teamcode.RR2.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.RR2.RR2;

public class AutoBase extends LinearOpMode {

    public TensorFlowMineralDetection tensorFlowMineralDetection;
    public RR2 robot;

    public void initLogic(){
        //Init's robot
        tensorFlowMineralDetection = new TensorFlowMineralDetection(hardwareMap,telemetry,this);
        robot = new RR2(hardwareMap,telemetry,this);

        robot.setBrakeModeDriveMotors();

        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tensorFlowMineralDetection.initVuforia();
        tensorFlowMineralDetection.initTfod();

        waitForStart();
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
        robot.intializeIMU();

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
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
            robot.finalTurn(-leftRightAngle);
            robot.finalMove(0.75, 58);
        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
            robot.finalTurn(leftRightAngle);
            robot.finalMove(0.75, 58);
        } else {
            robot.finalMove(0.75, 53);
        }
    }
}
