package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto.TensorFlowMineralDetection;
import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.RR2;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class AutoBase extends LinearOpMode {

    public TensorFlowMineralDetection tensorFlowMineralDetection;
    public Robot robot;

    public void initLogic(){
        //Init's robot
        robot = new Robot(hardwareMap,telemetry,this);

        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
    }

    @Override
    public void runOpMode(){
    }
}


