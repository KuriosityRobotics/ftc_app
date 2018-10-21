package org.firstinspires.ftc.teamcode.Tempest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Template", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
//@Disabled
public class TemplateMainTeleop extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
        //Init's robot
        Tempest robot = new Tempest(hardwareMap, telemetry, this);   //DO NOT DELETE

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            //when you hit start, code in this runs
        }
    }
}