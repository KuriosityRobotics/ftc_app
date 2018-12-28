package org.firstinspires.ftc.teamcode.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.RR2;

@Autonomous(name="Depot Facing 65 Point", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
public class DepotFacing65Point extends AutoBase
{

    @Override
    public void runOpMode(){
        //Init's robot
        initLogic();
        while (opModeIsActive()){
            dropDownFromLander();
            knockOffMineral();
            navigateToCrater();
            break;
        }
    }
    private void navigateToCrater() {

        robot.goToWall(0.3,40);
        robot.finalTurn(-38);
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
//            robot.finalTurn(-7);
//            robot.finalMove(0.5, 50);
//            robot.intake.setPower(1);
//            sleep(2500);
//            robot.intake.setPower(0);
//            robot.finalMove(0.5, -50);
//            robot.finalTurn(30);
            //Getting to Depot
        } else {
            robot.finalMove(0.5, 105);
            robot.intake.setPower(1);
            sleep(2500);
            robot.intake.setPower(0);
            robot.finalMove(0.5, -100);
            robot.finalTurn(70);
            //Getting to Depot
        }
    }
}