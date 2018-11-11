package org.firstinspires.ftc.teamcode.RR2;
/**
 * Created by Khue on 11/10/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MainTeleOp", group="Linear Opmode")
//@Disabled
public class MainTeleop extends LinearOpMode {

    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;
    int pivotPosition;
    public static double powerScaleFactor = 0.4;
    boolean hookIn;
    boolean onHook = true;
    long startTime = 0;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Init's robot
        RR2 robot = new RR2(hardwareMap, telemetry, this);   //DO NOT DELETE
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            //Drive Controls
            robot.fLeft.setPower(-gamepad1.left_stick_y);
            robot.bLeft.setPower(-gamepad1.left_stick_y);
            robot.fRight.setPower(-gamepad1.right_stick_y);
            robot.bRight.setPower(-gamepad1.right_stick_y);
            if (gamepad1.dpad_left) {
                robot.fLeft.setPower(1);
                robot.bLeft.setPower(1);
            }
            if (gamepad1.dpad_right) {
                robot.fRight.setPower(1);
                robot.bRight.setPower(1);
            }
            else {
                robot.allWheelDrive(0,0,0,0);
            }
            if (gamepad1.dpad_up) {
                robot.allWheelDrive(1,1,1,1);
            }
            if (gamepad1.dpad_down) {
                robot.allWheelDrive(-1,-1,-1,-1);
            }
            //Intake Control
            if (gamepad2.left_stick_y != 0) {
                robot.intake.setPower(gamepad2.left_stick_y);
                telemetry.addLine("Moving Intake");
                telemetry.update();
            } else {
                robot.intake.setPower(0);
                telemetry.addLine("Not Moving Intake");
                telemetry.update();
            }
            if (gamepad1.a) {
                robot.blocker.setPosition(0.7);
            }
            else if (gamepad1.b) {
                robot.blocker.setPosition(0.5);
            }
            //Pivoting Slide For Outtake
            if (gamepad1.y) {
                robot.pivot.setPower(-1);
                telemetry.addLine("Pivot Going Up");
                telemetry.update();
            }
            else if (gamepad1.x) {
                robot.pivot.setPower(1);
                telemetry.addLine("Pivot Going Down");
                telemetry.update();
            } else {
                robot.pivot.setPower(0);
            }

            //Controlling the Slide with Gamepad2
            if (gamepad2.right_stick_y != 0) {
                robot.slide.setPower(-gamepad2.right_stick_y);
            }
            else {
                robot.slide.setPower(0);
            }

            //Hang Locking
            if (gamepad2.a) {
                robot.hangLockLeft.setPosition(0.55);
                robot.hangLockRight.setPosition(0.3);
            }
            else if (gamepad2.b) {
                robot.hangLockLeft.setPosition(0.71);
                robot.hangLockRight.setPosition(0.21);
            }
            if (gamepad2.x) {
                robot.hook.setPower(0.3);
            } else if (gamepad2.y) {
                robot.hook.setPower(-0.3);
            } else {
                robot.hook.setPower(0);
            }

//            if(gamepad2.b && !hookIn && onHook){
//                robot.hangLockLeft.setPosition(90);
//                robot.hangLockRight.setPosition(90);
//                onHook = false;
//                hookIn = true;
//            }   else if (gamepad2.b && hookIn && !onHook) {
//                robot.hangLockRight.setPosition(0);
//                robot.hangLockLeft.setPosition(0);
//                onHook = true;
//                hookIn = true;
//            }
//            else if(!gamepad2.b){
//                hookIn = false;
//            }
        }
    }
}