package org.firstinspires.ftc.teamcode.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Depot Facing 65 Point", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
public class DepotFacing65Point extends AutoBase
{

    @Override
    public void runOpMode(){
        //Init's robot
        initLogic();
        while (opModeIsActive()){
            dropDownFromLander();
            knockOffMineral(35);
            navigateToCrater();
            break;
        }
    }

    private void navigateToCrater() {
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
            robot.finalMove(0.5, -53);
            robot.finalTurn(74);
            //Getting to Depot
        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.CENTER){
            robot.finalMove(0.5, 52);
            robot.releaseTeamMarker();
            robot.finalMove(0.5, -100);
            robot.finalTurn(70);
        }

        robot.goToWall(0.3,40);
        robot.finalTurn(-38);
        robot.goToCrater(-0.5);

        telemetry.addData("Status","done");
        telemetry.update();
    }
}