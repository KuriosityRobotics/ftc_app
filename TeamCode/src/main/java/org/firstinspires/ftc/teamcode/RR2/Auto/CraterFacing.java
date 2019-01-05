package org.firstinspires.ftc.teamcode.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.RR2;

@Autonomous(name="Crater Facing", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
public class CraterFacing extends AutoBase
{
    @Override
    public void runOpMode(){
        //Init's robot
        initLogic();
        while (opModeIsActive()){
            dropDownFromLander();
            knockOffMineral(40);
            navigateToDepotThenCrater();
            break;
        }
    }

    protected void navigateToDepotThenCrater() {
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.CENTER){
            robot.finalMove(0.5, -48);
            //Getting to Depot
        }else {
            robot.finalMove(0.5, -55);
            //Getting to Depot
        }

        robot.finalTurn(70);
        robot.goToWall(0.3,38);
        robot.finalTurn(135);
        robot.moveRobotKillSwitch(0.7,120,-120);
        robot.goToCrater(-0.7);
        telemetry.addData("Status","done");
        telemetry.update();
    }

//    protected void knockOffMineral() {
//        objectDetection();
//        telemetry.addLine("Mineral location: "+ tensorFlowMineralDetection.location);
//        telemetry.update();
//        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
//            robot.finalTurn(-40);
//            robot.finalMove(0.5, 58);
//            robot.finalMove(0.5, -55);
//            //Getting to Depot
//        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
//            robot.finalTurn(40);
//            robot.finalMove(0.5, 58);
//            robot.finalMove(0.5, -55);
//            //Getting to Depot
//        } else {
//            robot.finalMove(0.5, 53);
//            robot.finalMove(0.5, -48);
//            //Getting to Depot
//        }
//    }

}



