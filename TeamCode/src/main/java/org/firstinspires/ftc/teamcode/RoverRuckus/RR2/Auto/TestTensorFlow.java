package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Tensor FLow", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
@Deprecated
public class TestTensorFlow extends AutoBase
{
    @Override
    public void runOpMode(){
        //Init's robot
        initLogic();
        while (opModeIsActive()){
            telemetry.addLine(tensorFlowMineralDetection.runObjectDetection().toString());
            telemetry.update();
            sleep(5000);
        }
    }
}



