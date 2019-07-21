package org.firstinspires.ftc.teamcode.Skystone.MotionProfiler;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Skystone.MathFunctions;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class RobotMovementGenerator {
    public double xPos;
    public double yPos;
    public double angle;
    public RobotMovementGenerator(Robot robot){
        Position2D position2D = new Position2D(robot);
        position2D.runOnUiThread();
        position2D.getxPose();
        position2D.getyPose();
        this.xPos = position2D.getxPose();
        this.yPos = position2D.getyPose();
        this.angle = position2D.getAnglePose();
    }

    /*
    This needs to be put in a loop, it will generate the x, y and turn speeds that will be plugged into inverse kinematic equation in Robot to move
    * */
    public double[] toPoint(double x, double y, double speed, double turnSpeed){
        double xPos = this.xPos;
        double yPos = this.yPos;
        double anglePos = this.angle;

        double distanceToTarget = Math.hypot(x-xPos, y-yPos);
        double absoluteAngleToTarget = Math.atan2(y - xPos, x - xPos);

        double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - Math.toRadians(anglePos));
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(90);

        double xPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

        double xMovement = xPower * speed;
        double yMovement = yPower * speed;
        double turnMovement = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1);

        if(distanceToTarget < 10) {
            turnMovement = 0;
        }

        //plug xMovement, yMovement, turnMovement into inverse kinematics to move
        return new double[]{xMovement, yMovement, turnMovement};
    }
}
