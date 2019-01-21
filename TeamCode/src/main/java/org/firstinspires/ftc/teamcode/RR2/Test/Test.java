package org.firstinspires.ftc.teamcode.RR2.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Test", group="Linear Opmode")
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() {
//        resetRobot();
        waitForStart();
//        runtime.reset();
        while (opModeIsActive()) {
           telemetry.addData("lol",".");
           telemetry.update();
        }
    }
}



