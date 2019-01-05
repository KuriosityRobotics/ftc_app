package org.firstinspires.ftc.teamcode.RR2.Templates;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.RR2.Auto.TensorFlowMineralDetection;
import org.firstinspires.ftc.teamcode.RR2.RR2;

@Autonomous(name="Crater Facing", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
@Disabled //COMMENT OUT TO MAKE PROGRAM APPEAR ON PHONE
public class TemplateAuto extends AutoBase
{
    //IMPORTANT FOR THIS TO WORK MOVE THIS CLASS TO AUTO PACKAGE
    @Override
    public void runOpMode(){
        //Init's robot
        initLogic();
        while (opModeIsActive()){
            //code here when start button is pressed
            break;
        }
    }
}



