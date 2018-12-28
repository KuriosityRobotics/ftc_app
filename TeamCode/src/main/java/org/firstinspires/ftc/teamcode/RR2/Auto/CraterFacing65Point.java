package org.firstinspires.ftc.teamcode.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.RR2;

@Autonomous(name="Crater Facing 65 Point", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
public class CraterFacing65Point extends AutoBase
{
    @Override
    public void runOpMode(){
        //Init's robot
        initLogic();
        while (opModeIsActive()){
            dropDownFromLander();
            knockOffMineral();
            break;
        }
    }

    protected void knockOffMineral() {
        objectDetection();
        telemetry.addLine("Mineral location: "+ tensorFlowMineralDetection.location);
        telemetry.update();
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
            robot.finalTurn(-40);
            robot.finalMove(0.5, 60);
            robot.finalTurn(10);
            robot.finalMove(0.5,20);
            //Getting to Depot
        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
            robot.finalTurn(40);
            robot.finalMove(0.5, 60);
            robot.finalTurn(-10);
            robot.finalMove(0.5,20);
            //Getting to Depot
        } else {
            robot.finalMove(0.5,60);
            //Getting to Depot
        }
    }
}



