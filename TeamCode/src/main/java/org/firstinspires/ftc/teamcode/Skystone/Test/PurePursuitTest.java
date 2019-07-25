package org.firstinspires.ftc.teamcode.Skystone.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Skystone.CurvePoint;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

import java.util.Vector;

@Autonomous(name="PurePursuitTest ", group="Linear Opmode")
public class PurePursuitTest  extends LinearOpMode
{
    @Override
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap,telemetry,this);
        robot.driveMotorsBreakZeroBehavior();
        robot.resetEncoders();
        waitForStart();
        robot.intializeIMU();
        robot.changeRunModeToUsingEncoder();
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();

        while (opModeIsActive()){
            Vector<CurvePoint> allPoints = new Vector<>();
            allPoints.add(new CurvePoint(0,0,1,1,5,Math.toRadians(50),1));
            allPoints.add(new CurvePoint(20,0,1,1,5,Math.toRadians(50),1));
            allPoints.add(new CurvePoint(20,20,1,1,5,Math.toRadians(50),1));
            allPoints.add(new CurvePoint(40,20,1,1,5,Math.toRadians(50),1));

            robot.followCurve(allPoints,Math.toRadians(0));

            //robot.moveFollowCurve(allPoints);
//            break;
        }

    }
}

