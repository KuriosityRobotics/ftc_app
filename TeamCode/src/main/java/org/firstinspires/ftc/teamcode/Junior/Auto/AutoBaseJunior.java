package org.firstinspires.ftc.teamcode.Junior.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Junior.Junior;
import org.firstinspires.ftc.teamcode.RR2.RR2;

public class AutoBaseJunior extends LinearOpMode {

    public Junior robot;

    public void initLogic(){
        //Init's robot

        robot = new Junior(hardwareMap,telemetry,this);

        waitForStart();
    }

    @Override
    public void runOpMode(){

    }

}
