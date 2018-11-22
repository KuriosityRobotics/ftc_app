package org.firstinspires.ftc.teamcode.RR2;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Crater Facing", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
//@Disabled
public class CraterFacing extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
        //Init's robot
        TensorFlowMineralDetection tensorFlowMineralDetection = new TensorFlowMineralDetection(hardwareMap,telemetry,this);
        RR2 robot = new RR2(hardwareMap,telemetry,this);

        robot.fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intializeIMU();
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

            while(robot.distance.getDistance(DistanceUnit.MM)>150 && opModeIsActive()){
                telemetry.addData("encoder value of Pivot", robot.distance.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            telemetry.addData("done", "done");
            robot.pivot.setPower(0);

            robot.hangLockLeft.setPosition(0.71);
            robot.hangLockRight.setPosition(0.21);
            sleep(1000);

            robot.pivot.setPower(-1);

            robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.pivot.setTargetPosition(-4000);

            while(robot.pivot.isBusy() && opModeIsActive()){
                telemetry.addData("encoder value of Pivot", robot.pivot.getCurrentPosition());
                telemetry.update();
            }

            telemetry.update();
            robot.pivot.setPower(0);

            robot.hook.setPower(-0.35); //open
            sleep(2500);
            robot.hook.setPower(0);

            robot.moveRobot(0.2,50);
            robot.slide.setPower(0);
            tensorFlowMineralDetection.runObjectDetection();
            if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
                robot.finalTurn(-15);
                robot.allWheelDrive(0.5, 0.5, 0.5, 0.5);
                sleep(1500);
                robot.brakeRobot();
//                robot.moveRobot(0.2, 700);
//                robot.brakeRobot();
//                sleep(500);
//                robot.moveRobot(0.2, -500);
//                sleep(500);
//                robot.finalTurn(46);
                //Getting to Depot
            }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
                robot.finalTurn(15);
                robot.allWheelDrive(0.5, 0.5, 0.5, 0.5);
                sleep(1500);
                robot.brakeRobot();
//                sleep(500);
//                robot.brakeRobot();
//                sleep(500);
//                robot.moveRobot(0.2, -500);
//                sleep(500);
//                robot.finalTurn(46);
//                sleep(500);
                //Getting to Depot
            } else {
                robot.allWheelDrive(0.5, 0.5, 0.5, 0.5);
                sleep(1500);
                robot.brakeRobot();
//                robot.brakeRobot();
//                sleep(500);
//                robot.finalMove(0.2, -18);
//                robot.finalTurn(50);
                //Getting to Depot

            }
//            robot.finalMove(0.2, 27);
//            sleep(500);
//            robot.finalTurn(110);
//            //robot.intake.setPower(0.5);
//            //robot.slide.setPower();
//            robot.finalMove(0.2, 24);
//            sleep(500);
//            robot.finalMove(0.2, -24);
//            //after the robot turns u need to go foward and hit the mineral then go to depot and back to creater
//            //start programming here
//            //when you hit start, code in this runs
            sleep(1000000000);
        }
    }
}