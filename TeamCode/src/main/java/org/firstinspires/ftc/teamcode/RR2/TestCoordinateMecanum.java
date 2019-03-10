package org.firstinspires.ftc.teamcode.RR2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RR2.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.RR2.RR2;

@Autonomous(name="TestCoordinateMecanum", group="Linear Opmode")
public class TestCoordinateMecanum extends AutoBase {
        RR2 robot;
        @Override
        public void runOpMode() {
            robot = new RR2(hardwareMap,telemetry,this);
            waitForStart();
            while (opModeIsActive()) {
                robot.polarMovement(0.8,10, 45);

                break;
            }
        }
}
