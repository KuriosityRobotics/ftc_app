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
            telemetry.addData("fLeft",robot.fLeft.getCurrentPosition());
            telemetry.addData("bLeft",robot.bLeft.getCurrentPosition());
            telemetry.addData("fRight",robot.fRight.getCurrentPosition());
            telemetry.addData("bRight",robot.bRight.getCurrentPosition());
            telemetry.update();
//            dropDownFromLander();
//            knockOffMineral(45);
//            navigateToDepotThenCrater(distanceToDepot);
        }
    }

//    protected void navigateToDepotThenCrater(double distance) {
//        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
//            finalMove(0.5, -53);
//            finalTurn(74);
//            //Getting to Depot
//        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
//            distanceToDepot = 65;
//        } else {
//            center = true;
//            distance = 0;
//            //Getting to Depot
//            finalMove(0.5, 52);
//            releaseTeamMarker();
//            finalMove(0.5, -100);
//            finalTurn(70);
//        }
//
//        goToWall(0.3,25);
//        finalTurn(-38);
//        finalMove(0.5,distance);
//        if(!center) {
//            releaseTeamMarker();
//        }
//        goToCrater(-0.5);
//
//        telemetry.addData("Status","done");
//        telemetry.update();
//    }
}