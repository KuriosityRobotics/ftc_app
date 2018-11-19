package org.firstinspires.ftc.teamcode.RR2;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.RR2;
import org.firstinspires.ftc.teamcode.RR2.TensorFlowMineralDetection;

public class DepotFacing extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Init's robot
        TensorFlowMineralDetection tensorFlowMineralDetection = new TensorFlowMineralDetection(hardwareMap, telemetry, this);
        RR2 robot = new RR2(hardwareMap, telemetry, this);

        robot.fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intializeIMU();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            robot.pivot.setPower(1);

            while(robot.distance.getDistance(DistanceUnit.MM)>150 && opModeIsActive()){
                telemetry.addData("encoder value of Pivot", robot.distance.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            telemetry.addData("done", "done");
            robot.pivot.setPower(0);

            robot.hangLockLeft.setPosition(0.71);
            robot.hangLockRight.setPosition(0.21);
            sleep(1000);

            robot.pivot.setPower(-1);

            robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.pivot.setTargetPosition(-4000);

            while(robot.pivot.isBusy() && opModeIsActive()){
                telemetry.addData("encoder value of Pivot", robot.pivot.getCurrentPosition());
                telemetry.update();
            }

            telemetry.update();
            robot.pivot.setPower(0);

            robot.moveRobot(0.2,100);

            tensorFlowMineralDetection.runObjectDetection();
            if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
                robot.finalTurn(-20);
                robot.finalMove(0.2,19);
                robot.finalMove(0.2, -19);
            }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
                robot.finalTurn(20);
                robot.finalMove(0.2,19);
                robot.finalMove(0.2, -19);
            } else {
                robot.finalMove(0.2,19);
                robot.finalMove(0.2, -19);

            }
            robot.finalTurn(60);
            robot.finalMove(0.2, 47);
            robot.finalTurn(-30);
            robot.finalMove(0.2, 50);
            robot.intake.setPower(0.5);
            robot.finalMove(0.2, -75);
            sleep(1000000000);
        }

    }
}