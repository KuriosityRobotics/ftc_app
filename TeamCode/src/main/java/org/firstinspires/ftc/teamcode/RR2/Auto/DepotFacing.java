package org.firstinspires.ftc.teamcode.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.RR2;

@Autonomous(name="Depot Facing", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
public class DepotFacing extends AutoBase
{
    double distanceToDepot = 95;
    boolean center = false;
    @Override
    public void runOpMode(){
        initLogic();
        while (opModeIsActive()){
            dropDownFromLander();
            knockOffMineral(45);
            navigateToDepotThenCrater(distanceToDepot);
            break;
        }
    }

    protected void navigateToDepotThenCrater(double distance) {
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
            robot.finalMove(0.5, -53);
            robot.finalTurn(74);
            //Getting to Depot
        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
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
            center = true;
            robot.finalMove(0.5, 52);
            robot.releaseTeamMarker();
            robot.finalMove(0.5, -100);
            robot.finalTurn(70);
            //Getting to Depot
        }

        robot.goToWall(0.3,40);
        robot.finalTurn(-38);
        robot.finalMove(0.5,distance);
        if(!center) {
            robot.releaseTeamMarker();
        }
        robot.goToCrater(-0.5);

        telemetry.addData("Status","done");
        telemetry.update();
    }

    protected void knockOffMineral() {
        objectDetection();
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
            robot.finalTurn(-45);
            robot.finalMove(0.5, 60);
            robot.finalMove(0.5, -55);
            robot.finalTurn(74);
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
            center = true;
            robot.finalMove(0.5, 105);
            robot.releaseTeamMarker();
            robot.finalMove(0.5, -100);
            robot.finalTurn(70);
            //Getting to Depot
        }
    }
}