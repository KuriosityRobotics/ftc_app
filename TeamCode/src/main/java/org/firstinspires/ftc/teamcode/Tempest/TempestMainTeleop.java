package org.firstinspires.ftc.teamcode.Tempest;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Temepst: Main Teleop", group="Linear Opmode")
//@Disabled
public class TempestMainTeleop extends LinearOpMode
{
    public static double fLPower;
    public static double fRPower;
    public static double bLPower;
    public static double bRPower;

    public static double powerScaleFactor = 0.4;


    long startTime = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        //Init's robot
        Tempest robot = new Tempest(hardwareMap, telemetry, this);   //DO NOT DELETE

        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Clears power values
        fLPower = 0.0;
        fRPower = 0.0;
        bLPower = 0.0;
        bRPower = 0.0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            moveRobotLogic(robot,gamepad1,gamepad2,telemetry);

        }
    }

    public static void moveIntakeLogic(Tempest robot, Gamepad gamepad1, Gamepad gamepad2){

    }

    public static void moveRobotLogic(Tempest robot, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
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
            fLPower = (gamepad1.right_stick_y+powerScaleFactor);
            bLPower = (gamepad1.right_stick_y)+powerScaleFactor;
            fRPower = (gamepad1.left_stick_y)-powerScaleFactor;
            bRPower = (gamepad1.left_stick_y)-powerScaleFactor;
        } else if (gamepad1.dpad_left) {
            fRPower = (gamepad1.right_stick_y+powerScaleFactor);
            bRPower = (gamepad1.right_stick_y)+powerScaleFactor;
            fLPower = (gamepad1.left_stick_y)-powerScaleFactor;
            bLPower = (gamepad1.left_stick_y)-powerScaleFactor;
        }

        robot.fLeft(fLPower);
        robot.fRight(fRPower);
        robot.bLeft(bLPower);
        robot.bRight(bRPower);

        telemetry.update();
    }
}