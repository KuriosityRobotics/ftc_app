package org.firstinspires.ftc.teamcode.Tempest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Khue on 9/29/18.
 */

@TeleOp(name="RR2 Teleop", group="Linear Opmode")
//@Disabled
public class TempestTeleopKhue extends LinearOpMode {
    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

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
            if (gamepad1.left_stick_y != 0) {
                robot.fLeft(-gamepad1.left_stick_y);
                robot.bLeft(-gamepad1.left_stick_y);
            }
            if (gamepad1.right_stick_y != 0) {
                robot.fRight(-gamepad1.right_stick_y);
                robot.bRight(-gamepad1.right_stick_y);
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
                robot.lSlideUp.setPower(-0.8);
                robot.InTake();
            }else{
                robot.lSlideUp.setPower(0);
            }
            if (gamepad2.left_bumper) {
                robot.lSlideUp.setPower(0.8);
                robot.moveLinearSlideUp();
                robot.OutTake();
            }

            robot.lSlideRight.setPower(gamepad2.right_stick_y);
            robot.lSlideLeft.setPower(gamepad2.right_stick_y);

            telemetry.update();

        }
    }
}