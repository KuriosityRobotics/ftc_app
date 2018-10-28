package org.firstinspires.ftc.teamcode.Tempest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Template Teleop", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
//@Disabled
public class SamTeleop extends LinearOpMode
{
    double flPower = 0;
    double frPower = 0;
    double blPower = 0;
    double brPower = 0;

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
        //Init's robot
        Tempest robot = new Tempest(hardwareMap, telemetry, this);   //DO NOT DELETE

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            //when you hit start, code in this runs
            flPower = gamepad1.left_stick_y;
            blPower = gamepad1.left_stick_y;
            frPower = gamepad1.right_stick_y;
            brPower = gamepad1.right_stick_y;
            robot.fLeft.setPower(flPower);
            robot.bRight.setPower(brPower);
            robot.fRight.setPower(frPower);
            robot.bRight.setPower(brPower);
        }
    }
}