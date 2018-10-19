package org.firstinspires.ftc.teamcode.Tempest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Khue on 9/29/18.
 */

@TeleOp(name="RR2 Teleop", group="Linear Opmode")
//@Disabled
public class TempestTeleopKhue extends LinearOpMode {
    public  double fLPower;
    public double fRPower;
    public  double bLPower;
    public  double bRPower;

    public  double powerScaleFactor = 0.4;

    public static double intakePower = 0;
    public static double slidePower = 0;

    public static double intakeRightPosition = 0;
    public static double intakeLeftPosition = 0;

    long startTime = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        //Init's robot
        Tempest robot = new Tempest(hardwareMap, telemetry, this);   //DO NOT DELETE

        //Clears power values
        waitForStart();
        runtime.reset();
        robot.speedSet = 0.6;
        while (opModeIsActive()){
            if (gamepad1.right_stick_y != 0 || gamepad1.left_stick_y != 0) {
                robot.allWheelDrive(gamepad1.left_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.right_stick_y);
            } else {
                robot.setDrivePower(0);
            }
            if(gamepad1.dpad_up) {
                robot.setDrivePower(0.6);
            }else if (gamepad1.dpad_down) {
                robot.setDrivePower(-0.6);
            }else if (gamepad1.dpad_right) {
                robot.allWheelDrive(0.6, 0.6, -0.6, -0.6);
            }else if(gamepad1.dpad_left) {
                robot.allWheelDrive(-0.6, -0.6, 0.6, 0.6);
            }else{
                robot.setDrivePower(0);
            }
            if (gamepad2.right_bumper) {
                robot.moveLinearSlideDown();
                robot.pivot.setPower(-0.8);
                robot.InTake();
            }else{
                robot.pivot.setPower(0);
            }
            if (gamepad2.left_bumper) {
                robot.pivot.setPower(0.8);
                robot.moveLinearSlideUp();
                robot.OutTake();
            }

            robot.slideRight.setPower(gamepad2.right_stick_y);
            robot.slideLeft.setPower(gamepad2.right_stick_y);
            if(gamepad2.x){
                robot.intakeRight.setPosition(1);
                robot.intakeLeft.setPosition(0);
            }else if(gamepad2.y){
                robot.intakeRight.setPosition(0);
                robot.intakeLeft.setPosition(1);
            }
            pivotIntakeLogic(robot,gamepad1,gamepad2,telemetry);
            moveIntakeLogic(robot,gamepad1,gamepad2,telemetry);

            telemetry.update();

        }
    }

    public static void runTeleop(Tempest robot, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){
        pivotIntakeLogic(robot,gamepad1,gamepad2,telemetry);
        moveIntakeLogic(robot,gamepad1,gamepad2,telemetry);
    }

    public static void moveIntakeLogic(Tempest robot, Gamepad gamepad1, Gamepad gamepad2,Telemetry telemetry){
        robot.intakeRight.setPosition(0);
        telemetry.addData("position",robot.intakeLeft.getPosition());
//        robot.intakeRight.setPosition(0);

    }

    public static void pivotIntakeLogic(Tempest robot, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){
        if(gamepad2.x){
            intakePower = 1;
        }else if(gamepad2.y){
            intakePower = -1;
        }else{
            intakePower = 0;
        }
        robot.intake.setPower(intakePower);
    }
}