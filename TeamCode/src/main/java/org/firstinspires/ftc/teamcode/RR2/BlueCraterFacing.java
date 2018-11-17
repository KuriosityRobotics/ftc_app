package org.firstinspires.ftc.teamcode.RR2;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Crater Facing", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
//@Disabled
public class BlueCraterFacing extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
        //Init's robot
        TensorFlowMineralDetection tensorFlowMineralDetection = new TensorFlowMineralDetection(hardwareMap,telemetry,this);
        RR2 robot = new RR2(hardwareMap,telemetry,this);
        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
//            tensorFlowMineralDetection.runObjectDetection();
//            telemetry.addData("Location",tensorFlowMineralDetection.location);
//            robot.hook.setPower(-0.35); //open
//            sleep(2500);
//            robot.hook.setPower(0);
//            sleep(2000);
//            robot.hook.setPower(0.35); //close
//            sleep(1000);
//            robot.hook.setPower(0);

            robot.pivot.setPower(1);

            robot.pivot.setTargetPosition(600);

            while(robot.pivot.isBusy()){
                telemetry.addData("encoder value of Pivot", robot.pivot.getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData("done", "done");


            robot.hangLockLeft.setPosition(0.71);
            robot.hangLockRight.setPosition(0.21);
            sleep(1000);

            robot.pivot.setPower(-1);

            robot.pivot.setTargetPosition(-4000);

            while(robot.pivot.isBusy()){
                telemetry.addData("encoder value of Pivot", robot.pivot.getCurrentPosition());
                telemetry.update();
            }

            telemetry.update();
            robot.pivot.setPower(0);

            robot.hook.setPower(-0.35); //open
            sleep(2500);
            robot.hook.setPower(0);
            sleep(100000);
            //when you hit start, code in this runs
        }
    }
}