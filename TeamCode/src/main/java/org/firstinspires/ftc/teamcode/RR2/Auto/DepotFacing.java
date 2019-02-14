package org.firstinspires.ftc.teamcode.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        } else {
            center = true;
            distance = 0;
            //Getting to Depot
            robot.finalMove(0.5, 52);
            robot.releaseTeamMarker();
            robot.finalMove(0.5, -100);
            robot.finalTurn(70);
        }

        robot.goToWall(0.3,25);

        if(!center) {
            robot.finalTurn(-42);
            robot.finalMove(0.3,distance);
            robot.releaseTeamMarker();
            robot.goToCrater(-0.5);
        }
        if (center) {
            robot.finalTurn(132);
            robot.finalMove(0.5, 40);
            robot.slide.setTargetPosition(200);
            robot.slide.setPower(-0.7);
        }

        robot.goToCrater(-0.5);

        telemetry.addData("Status","done");
        telemetry.update();
    }
}