package org.firstinspires.ftc.teamcode.RR2;
/**
 * Created by Khue on 11/10/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="MainTeleOp", group="Linear Opmode")
//@Disabled
public class MainTeleop extends LinearOpMode {

    RR2 robot;
    boolean changedBlocker = false, onBlock = false;
    boolean changedHook = false, onHook = false;

    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;
    public static double powerScaleFactor = 1.5;
    long startTime = 0;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Init's robot
        robot = new RR2(hardwareMap, telemetry, this);   //DO NOT DELETE
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            driveLogic();
            intakeLogic();
            blockerLogic();
            pivotLogic();
            slideLogic();
            hangLogic();
            hookLogic();
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
        robot.intake.setPower(-gamepad2.left_stick_y);
    }

    private void blockerLogic(){
        //blocker for outtake
        if(gamepad2.x){
            robot.blocker.setPosition(0.5);
        }else{
            robot.blocker.setPosition(0.1);
        }
    }

    private void pivotLogic(){
        //Pivoting Slide For Outtake
        if (gamepad1.y) {
            robot.pivot.setPower(-1);
            telemetry.addLine("Pivot Going Up");
            telemetry.update();
        }
        else if (gamepad1.x && robot.distance.getDistance(DistanceUnit.MM)>150) {
            robot.pivot.setPower(1);
            telemetry.addLine("Pivot Going Down");
            telemetry.update();
        } else {
            robot.pivot.setPower(0);
        }
    }

    private void slideLogic(){
        //Controlling the Slide with Gamepad2
        robot.slide.setPower(-gamepad2.right_stick_y);
    }

    private void hangLogic(){
        //Hang Locking
        if (gamepad1.a) {
            robot.hangLockLeft.setPosition(0.40);
            robot.hangLockRight.setPosition(0.38);
        }
        else if (gamepad1.b) {
            robot.hangLockLeft.setPosition(0.61);
            robot.hangLockRight.setPosition(0.21);
        }
    }

    private void hookLogic(){
        if (gamepad2.a) {
            robot.hook.setPower(0.35);
            sleep(2500);
            robot.hook.setPower(0);
        }else if (gamepad2.b) {
            robot.hook.setPower(-0.35);
            sleep(1000);
            robot.hook.setPower(0);

        } else {
            robot.hook.setPower(0);
        }
    }
}


