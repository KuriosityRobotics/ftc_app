package org.firstinspires.ftc.teamcode.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.RR2;

@Autonomous(name="Depot Facing 65 Point", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
public class DepotFacing65Point extends LinearOpMode
{

    @Override
    public void runOpMode(){
        //Init's robot
//        initLogic();
        while (opModeIsActive()){
//            dropDownFromLander();
//            knockOffMineral(45);
//            navigateToCrater();
            break;
        }
    }
//    private void navigateToCrater() {
//
//        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
//            finalMove(0.5, -53);
//            finalTurn(74);
//            //Getting to Depot
//        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.CENTER){
//            finalMove(0.5, 52);
//            releaseTeamMarker();
//            finalMove(0.5, -100);
//            finalTurn(70);
//        }
//
//        goToWall(0.3,40);
//        finalTurn(-38);
//        goToCrater(-0.5);
//
//        telemetry.addData("Status","done");
//        telemetry.update();
//    }
}