package org.firstinspires.ftc.teamcode.Tempest;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="Template Auto", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
//@Disabled
public class TemplateAuto extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
        //Init's robot
        Tempest robot = new Tempest(hardwareMap, telemetry, this);   //DO NOT DELETE
        TensorFlowOpMode tensorFlowOpMode = new TensorFlowOpMode(hardwareMap,telemetry,this);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            if(tensorFlowOpMode.runObjectDetection() == TensorFlowOpMode.Location.CENTER){
                telemetry.addData("Mineral","Center");
            }else if(tensorFlowOpMode.runObjectDetection() == TensorFlowOpMode.Location.LEFT){
                telemetry.addData("Mineral","Left");
            }else{
                telemetry.addData("Mineral","Right");
            }
            telemetry.update();
            //when you hit start, code in this runs
            sleep(100000); //this should always be the last line of your code don't delete this
        }
    }
}