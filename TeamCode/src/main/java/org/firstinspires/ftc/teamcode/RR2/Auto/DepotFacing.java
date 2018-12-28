package org.firstinspires.ftc.teamcode.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.RR2;

@Autonomous(name="Depot Facing", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
public class DepotFacing extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    TensorFlowMineralDetection tensorFlowMineralDetection;

    double distanceToDepot;
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

        tensorFlowMineralDetection.initVuforia();
        tensorFlowMineralDetection.initTfod();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            dropDownFromLander();
            knockOffMineral();
            navigateToDepotThenCrater(distanceToDepot);
            break;
        }
    }
    private void navigateToDepotThenCrater(double distance) {

        robot.goToWall(0.3,40);
        robot.finalTurn(-38);
        robot.finalMove(0.5,distance);
        robot.goToCrater(-0.5);

        telemetry.addData("Status","done");
        telemetry.update();
    }

    private void knockOffMineral() {
        robot.intializeIMU();
        robot.moveRobot(0.5,50);
        robot.slide.setPower(0);
        tensorFlowMineralDetection.runObjectDetection();
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
            robot.finalTurn(-45);
            robot.finalMove(0.5, 60);
            robot.finalMove(0.5, -55);
            robot.finalTurn(74);
            distanceToDepot = 95;
            //Getting to Depot
        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
            robot.finalTurn(45);
            distanceToDepot = 65;
//            robot.finalTurn(-7);
//            robot.finalMove(0.5, 50);
//            robot.intake.setPower(1);
//            sleep(2500);
//            robot.intake.setPower(0);
//            robot.finalMove(0.5, -50);
//            robot.finalTurn(30);
            //Getting to Depot
        } else {
            distanceToDepot = 95;
            robot.finalMove(0.5, 105);
            robot.intake.setPower(1);
            sleep(2500);
            robot.intake.setPower(0);
            robot.finalMove(0.5, -100);
            robot.finalTurn(70);
            //Getting to Depot
        }
    }

    private void dropDownFromLander(){
        robot.pivot.setPower(-1);
        robot.pivot2.setPower(1);

        while(robot.distance.getDistance(DistanceUnit.MM)>150 && opModeIsActive()){
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

        while(robot.bottomDistance.getDistance(DistanceUnit.MM) >23 && opModeIsActive()){
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
        while(robot.distance.getDistance(DistanceUnit.MM)>150 && opModeIsActive()){
            telemetry.addData("encoder value of Pivot", robot.distance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        robot.pivot2.setPower(0);
        robot.pivot.setPower(0);
    }
}