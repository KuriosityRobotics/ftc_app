package org.firstinspires.ftc.teamcode.RR2;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Template Auto", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
//@Disabled
public class TemplateAuto extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
        //Init's robot
        TensorFlowMineralDetection tensorFlowMineralDetection = new TensorFlowMineralDetection(hardwareMap,telemetry,this);
        RR2 robot = new RR2(hardwareMap,telemetry,this);
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            telemetry.addData("distance",robot.distance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}