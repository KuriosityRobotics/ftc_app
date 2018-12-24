package org.firstinspires.ftc.teamcode.RR2;

import android.content.Intent;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="MainTeleOp", group="Linear Opmode")
public class MainTeleop extends LinearOpMode {

    RR2 robot;

    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

    double intakePower;
    boolean onSlowDrive, changedSlowDrive = false;
    public static double powerScaleFactor = 1;
    long startTime = 0;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Init's robot
        robot = new RR2(hardwareMap, telemetry, this);   //DO NOT DELETE
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            slowDriveLogic();
            driveLogic();
            intakeLogic();
            blockerLogic();
            pivotLogic();
            slideLogic();
            hangLogic();
            hookLogic();
            setToHangMode();
            hangRobot();
            hangRobot();
            dropRobot();
            movePivotToDumpPosition();
        }
    }

    //teleop methods
    private void driveLogic(){
        //tank drive
        fLPower = -(gamepad1.left_stick_y)*powerScaleFactor;
        bLPower = -(gamepad1.left_stick_y)*powerScaleFactor;
        fRPower = -(gamepad1.right_stick_y)*powerScaleFactor;
        bRPower = -(gamepad1.right_stick_y)*powerScaleFactor;
        //Straight D-Pad move
        if (gamepad1.dpad_up) {
            fLPower = (gamepad1.left_stick_y)+powerScaleFactor;
            bLPower = (gamepad1.left_stick_y)+powerScaleFactor;
            fRPower = (gamepad1.right_stick_y+powerScaleFactor);
            bRPower = (gamepad1.right_stick_y+powerScaleFactor);
        } else if (gamepad1.dpad_down) {
            fLPower = (gamepad1.left_stick_y)-powerScaleFactor;
            bLPower = (gamepad1.left_stick_y)-powerScaleFactor;
            fRPower = (gamepad1.right_stick_y-powerScaleFactor);
            bRPower = (gamepad1.right_stick_y)-powerScaleFactor;
        } else if (gamepad1.dpad_right) {
            fLPower = (gamepad1.right_stick_y)+powerScaleFactor;
            bLPower = (gamepad1.right_stick_y)+powerScaleFactor;
            fRPower = (gamepad1.left_stick_y)-powerScaleFactor;
            bRPower = (gamepad1.left_stick_y)-powerScaleFactor;
        } else if (gamepad1.dpad_left) {
            fRPower = (gamepad1.right_stick_y)+powerScaleFactor;
            bRPower = (gamepad1.right_stick_y)+powerScaleFactor;
            fLPower = (gamepad1.left_stick_y)-powerScaleFactor;
            bLPower = (gamepad1.left_stick_y)-powerScaleFactor;
        }


        robot.fLeft.setPower(fLPower);
        robot.fRight.setPower(fRPower);
        robot.bLeft.setPower(bLPower);
        robot.bRight.setPower(bRPower);
    }

    private void intakeLogic(){
        //Intake Control
        robot.intake.setPower(intakePower);
    }

    private void blockerLogic(){
        //blocker for outtake
        if(gamepad2.x){
            robot.blocker.setPosition(0.5);
            intakePower = -1;
        }else{
            intakePower = -gamepad2.left_stick_y;
            robot.blocker.setPosition(0.1);
        }
    }

    private void pivotLogic(){
        //Pivoting Slide For Outtake
        if (gamepad1.y) {
            robot.pivot.setPower(-1);
            robot.pivot2.setPower(1);
        }
        else if (gamepad1.x && robot.distance.getDistance(DistanceUnit.MM)>150) {
            robot.pivot.setPower(1);
            robot.pivot2.setPower(-1);
        } else {
            robot.pivot.setPower(0);
            robot.pivot2.setPower(0);
        }
    }

    private void slideLogic(){
        //Controlling the Slide with Gamepad2
        double slidePower = gamepad2.right_stick_y;
        robot.slide.setPower(slidePower);
    }

    private void hangLogic(){
        //Hang Locking
        if (gamepad2.left_bumper) {
            robot.hangLockClose();
        }
        else if (gamepad2.right_bumper) {
            robot.hangLockOpen();
        }
    }

    private void hookLogic(){
        if (gamepad2.a) {
            robot.hook.setPosition(0);
        }else if (gamepad2.b) {
            robot.hook.setPosition(1);
        }
    }

    private void slowDriveLogic(){
        if(powerScaleFactor == 0.3){
            telemetry.addData("Driving Mode","Slow");
        }else{
            telemetry.addData("Driving Mode","Normal");
        }
        if(gamepad1.left_bumper && !changedSlowDrive){
            powerScaleFactor = (onSlowDrive) ? 1 : 0.3;
            onSlowDrive = !onSlowDrive;
            changedSlowDrive = true;
        }else if(!gamepad1.left_bumper){
            changedSlowDrive = false;
        }
        telemetry.update();
    }



    private void setToHangMode(){
        if(gamepad1.a) {
            robot.hangLockOpen();
            robot.hook.setPosition(0);
            telemetry.addData("hook status","opening...");
            telemetry.update();
            robot.pivot.setPower(-1);
            robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.pivot.setTargetPosition(-4500);

            while(robot.pivot.isBusy() && opModeIsActive()){
                driveLogic();
            }
            robot.pivot.setPower(0);
            robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void hangRobot(){
        if(gamepad1.b){
            robot.hook.setPosition(1);
            sleep(1000);
            robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pivot.setPower(1);
            while(robot.distance.getDistance(DistanceUnit.MM)>70 && opModeIsActive()){
                if(gamepad2.right_bumper){
                    telemetry.addData("Hang Status","Aborting hang...");
                    telemetry.update();
                    return;
                }else{
                    telemetry.addData("Hang Status","Hanging...");
                    telemetry.update();
                }
            }
            sleep(500);
            robot.pivot.setPower(0);
            robot.hangLockClose();
            telemetry.addData("Hang Status","Hang successful");
            telemetry.update();
        }
    }

    private void dropRobot(){
        if(gamepad1.left_bumper && gamepad1.right_bumper){
            robot.pivot.setPower(1);

            while(robot.distance.getDistance(DistanceUnit.MM)>150 && opModeIsActive()){
                telemetry.addData("encoder value of Pivot", robot.distance.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            telemetry.addData("done", "done");
            robot.pivot.setPower(0);

            robot.hangLockOpen();
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

            robot.hook.setPosition(0); //open

            robot.pivot.setPower(1);
            robot.pivot.setTargetPosition(0);
        }
    }

    private void movePivotToDumpPosition(){
        if(gamepad1.right_trigger>0){
            robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.pivot.setPower(-1);
            robot.pivot.setTargetPosition(-4750);
            while (robot.pivot.isBusy() && opModeIsActive()){
                driveLogic();
                slideLogic();
                intakeLogic();
            }
            robot.pivot.setPower(0);
            robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}



