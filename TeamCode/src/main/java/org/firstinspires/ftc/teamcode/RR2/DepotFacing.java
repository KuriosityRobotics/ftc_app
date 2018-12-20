package org.firstinspires.ftc.teamcode.RR2;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Depot Facing", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
public class DepotFacing extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    TensorFlowMineralDetection tensorFlowMineralDetection;
    RR2 robot;
    @Override
    public void runOpMode(){
        //Init's robot
        tensorFlowMineralDetection = new TensorFlowMineralDetection(hardwareMap,telemetry,this);
        robot = new RR2(hardwareMap,telemetry,this);

        robot.fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            dropDownFromLander();
            knockOffMineral();
            navigateToDepotThenCrater();
            break;
        }
    }
    private void navigateToDepotThenCrater() {
        robot.finalTurn(60);

        robot.finalMove(0.5, 94);
        robot.finalTurn(135);
        //robot.intake.setPower(0.5);
        //robot.slide.setPower();
        robot.finalMove(0.7, 75);
        telemetry.addData("Status","done");
        telemetry.update();
    }

    private void knockOffMineral() {
        robot.intializeIMU();
        robot.moveRobot(0.5,50);
        robot.slide.setPower(0);
        tensorFlowMineralDetection.runObjectDetection();
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
            robot.finalTurn(-22);
            robot.finalMove(0.5, 60);
            robot.finalTurn(12);
            robot.finalMove(0.5, 50);
            robot.intake.setPower(1);
            sleep(2500);
            robot.intake.setPower(0);
            robot.finalMove(0.5, -50);
            robot.finalTurn(-22);
            robot.finalMove(0.5, -55);
            //Getting to Depot
        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
            robot.finalTurn(22);
            robot.finalMove(0.5, 60);
            robot.finalTurn(-12);
            robot.finalMove(0.5, 50);
            robot.intake.setPower(1);
            sleep(2500);
            robot.intake.setPower(0);
            robot.finalMove(0.5, -50);
            robot.finalTurn(22);
            robot.finalMove(0.5, -55);
            //Getting to Depot
        } else {
            robot.finalMove(0.5, 105);
            robot.intake.setPower(1);
            sleep(2500);
            robot.intake.setPower(0);
            robot.finalMove(0.5, -100);
            //Getting to Depot
        }
    }

    private void dropDownFromLander(){
        robot.pivot.setPower(1);

        while(robot.distance.getDistance(DistanceUnit.MM)>150 && opModeIsActive()){
            telemetry.addData("encoder value of Pivot", robot.distance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        telemetry.addData("done", "done");
        robot.pivot.setPower(0);

        robot.hangLockOpen();
        sleep(1000);

        robot.pivot.setPower(-1);

        while(robot.bottomDistance.getDistance(DistanceUnit.MM) >23 && opModeIsActive()){
            telemetry.addData("encoder value of Pivot", robot.pivot.getCurrentPosition());
            telemetry.update();
        }

        telemetry.update();
        robot.pivot.setPower(0);

        robot.hook.setPosition(0); //open
        sleep(1000);
        robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.pivot.setPower(1);
        robot.pivot.setTargetPosition(0);
    }
}