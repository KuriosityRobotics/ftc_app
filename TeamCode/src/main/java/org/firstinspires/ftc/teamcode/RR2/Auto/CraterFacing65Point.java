package org.firstinspires.ftc.teamcode.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.RR2;

@Autonomous(name="Crater Facing 65 Point", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
public class CraterFacing65Point extends LinearOpMode
{
    @Override
    public void runOpMode(){
        //Init's robot
//        initLogic();
        while (opModeIsActive()){
//            dropDownFromLander();
//            knockOffMineral(40);
//            navigateToCrater();
            break;
        }
    }

//    protected void navigateToCrater(){
//        //go to crater
//        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
//            craterTurn(10);
//        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
//            craterTurn(-10);
//        } else {
//            finalMove(0.5,60);
//        }
//    }
//
//    protected void craterTurn(double angle){
//        finalMove(0.5, 60);
//        finalTurn(angle);
//        finalMove(0.5,20);
//    }
}



