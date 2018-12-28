package org.firstinspires.ftc.teamcode.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.RR2;

@Autonomous(name="Crater Facing", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
public class CraterFacing extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private RR2 robot;
    private TensorFlowMineralDetection tensorFlowMineralDetection;
    @Override
    public void runOpMode() throws InterruptedException{
        //Init's robot
        tensorFlowMineralDetection = new TensorFlowMineralDetection(hardwareMap,telemetry,this);
        robot = new RR2(hardwareMap,telemetry,this);

        robot.fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.pivot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tensorFlowMineralDetection.initVuforia();
        tensorFlowMineralDetection.initTfod();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){

//            robot.goToWall(0.3);
//            break;
//            telemetry.addData("frontRightDistance",robot.frontRightDistance.getDistance(MM));
//            telemetry.addData("backRightDistance",robot.backRightDistance.getDistance(MM));
//            telemetry.update();
            dropDownFromLander();
            knockOffMineral();
            navigateToDepotThenCrater();
//            telemetry.addLine("Mineral location: "+ tensorFlowMineralDetection.runObjectDetection().toString());
//            telemetry.update();
//            sleep(1000);
//            robot.moveRobotKillSwitch(0.7,120,-120);
//            robot.keepDistance(10000);
            break;
        }
    }

    private void navigateToDepotThenCrater() {
        robot.finalTurn(70);
        robot.goToWall(0.3,38);
        robot.finalTurn(135);
        robot.moveRobotKillSwitch(0.7,120,-120);
        robot.goToCrater(-0.7);
        telemetry.addData("Status","done");
        telemetry.update();
    }

    private void knockOffMineral() {
        robot.intializeIMU();
        robot.moveRobot(0.5,50);
        robot.slide.setPower(0);
        tensorFlowMineralDetection.runObjectDetection();
        telemetry.addLine("Mineral location: "+ tensorFlowMineralDetection.location);
        telemetry.update();
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
            robot.finalTurn(-40);
            robot.finalMove(0.5, 58);
            robot.finalMove(0.5, -53);
            //Getting to Depot
        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
            robot.finalTurn(40);
            robot.finalMove(0.5, 58);
            robot.finalMove(0.5, -55);
            //Getting to Depot
        } else {
            robot.finalMove(0.5, 53);
            robot.finalMove(0.5, -48);
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



