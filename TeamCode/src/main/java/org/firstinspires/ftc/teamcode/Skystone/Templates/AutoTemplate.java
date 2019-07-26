package org.firstinspires.ftc.teamcode.Skystone.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Skystone.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

@Autonomous(name="PurePursuitTest ", group="Linear Opmode")
public class AutoTemplate extends AutoBase
{
    public static double[][] testPoints = {{0,-20},{-30,-65},{0,-35}};
    public static double[][] testPoints2 = {{0,-20},{-30,-65},{0,-35}};
    @Override
    public void runOpMode(){
        initLogic();

        PathPoints testPath = new PathPoints(testPoints);

        while (opModeIsActive()){
            robot.moveFollowCurve(testPath.targetPoints);
            //does a new path below
            robot.moveFollowCurve(testPath.newPoints(testPoints2));
            break;
        }
    }
}

