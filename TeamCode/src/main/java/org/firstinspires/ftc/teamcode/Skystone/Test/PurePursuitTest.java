package org.firstinspires.ftc.teamcode.Skystone.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.SplineInterpolate;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

@Autonomous(name="PurePursuitTest ", group="Linear Opmode")
public class PurePursuitTest  extends LinearOpMode
{
    public static double[][] testPoints = {{0,-20},{-30,-65}};

    @Override
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap,telemetry,this);
        robot.driveMotorsBreakZeroBehavior();
        robot.resetEncoders();
        SplineInterpolate s = new SplineInterpolate(testPoints);

        waitForStart();
        robot.intializeIMU();
        robot.changeRunModeToUsingEncoder();

        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();
        PathPoints testPath = new PathPoints(s.getPoints());

        while (opModeIsActive()){
//            robot.followCurve(testPath.targetPoints,Math.toRadians(0));
            robot.moveFollowCurve(testPath.targetPoints);
            sleep(5000);
            break;
        }
    }
}