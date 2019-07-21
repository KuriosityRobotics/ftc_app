package org.firstinspires.ftc.teamcode.Skystone.Odometry;

import android.os.Looper;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

import static org.firstinspires.ftc.teamcode.Skystone.MathFunctions.AngleWrap;

class MyThread extends Thread{
    boolean running;
    @Override
    public void run() {
        while (running){

        }
    }
}

public class Position2D{

    double xPose;
    Odometry odometry;
    double yPose;
    double anglePose;
    FtcRobotControllerActivity activity;
    public Position2D(Robot robot) {
        final Odometry v = new Odometry(robot);
        odometry = v;
    }

    public void runOnUiThread(){

        activity = (FtcRobotControllerActivity) AppUtil.getInstance().getRootActivity();

        activity.runOnUiThread(new Runnable() {
            public void run() {
                try {
                    odometry.constantVelocityOdometry();
                    xPose = odometry.xPosGlobal;
                    yPose = odometry.yPosGlobal;
                    anglePose = odometry.angleGlobal;
                    Looper.prepare();
                    Looper.loop();
                }catch (Exception e){

                }
            }
        });
    }

    public double getxPose() { return xPose; }

    public double getyPose() { return yPose; }

    public double getAnglePose() { return Math.toDegrees(AngleWrap(anglePose)); }
}

class Odometry{

    public Robot robot;

    double xPosGlobal = 0;
    double yPosGlobal = 0;
    double angleGlobal = 0;

    double angleDeltaRobot;
    double xDeltaRobot;
    double yDeltaRobot;

    double fLeftNEW = 0;
    double fRightNEW = 0;
    double bLeftNEW = 0;
    double bRightNEW = 0;

    double fLeftOLD = 0;
    double fRightOLD = 0;
    double bLeftOLD = 0;
    double bRightOLD = 0;

    double fl;
    double fr;
    double bl;
    double br;

    double wheelRadius = 2;
    final double wheelCircumference = 4 * Math.PI;
    final double encoderPerRevolution = 806.4;
    final double l = 7;
    final double w = 6.5;

    public Odometry(Robot robot){
        this.robot = robot;
    }
    public void constantVelocityOdometry() {

        fLeftNEW = robot.fLeft.getCurrentPosition();
        fRightNEW = robot.fRight.getCurrentPosition();
        bLeftNEW = robot.bLeft.getCurrentPosition();
        bRightNEW = robot.bRight.getCurrentPosition();

        // find robot position
        fl = 2 * Math.PI * (fLeftNEW - fLeftOLD) / encoderPerRevolution;
        fr = 2 * Math.PI * (fRightNEW - fRightOLD) / encoderPerRevolution;
        bl = 2 * Math.PI * (bLeftNEW - bLeftOLD) / encoderPerRevolution;
        br = 2 * Math.PI * (bRightNEW - bRightOLD) / encoderPerRevolution;

        xDeltaRobot = wheelRadius /4 * (fl + bl + br + fr);
        yDeltaRobot = wheelRadius /4 * (-fl + bl - br + fr);
        angleDeltaRobot = wheelRadius /4 *(-fl/(l+w) - bl/(l+w) + br/(l+w) + fr/(l+w));

        //converting to global frame
        if (angleDeltaRobot == 0){
            xPosGlobal += xDeltaRobot * Math.cos(angleGlobal) - yDeltaRobot * Math.sin(angleGlobal);
            yPosGlobal += xDeltaRobot * Math.sin(angleGlobal) + yDeltaRobot * Math.cos(angleGlobal);

        } else {
            xPosGlobal += (Math.cos(angleGlobal) * Math.sin(angleDeltaRobot) - (Math.cos(angleDeltaRobot) - 1) * Math.sin(angleGlobal)) * xDeltaRobot / angleDeltaRobot + (Math.cos(angleGlobal) * (Math.cos(angleDeltaRobot) - 1) - Math.sin(angleGlobal) * Math.sin(angleDeltaRobot)) * yDeltaRobot / angleDeltaRobot;
            yPosGlobal += ((Math.cos(angleDeltaRobot) - 1) * Math.sin(angleGlobal) + (Math.cos(angleGlobal)) * Math.sin(angleDeltaRobot)) * yDeltaRobot / angleDeltaRobot + (Math.cos(angleGlobal) * (Math.cos(angleDeltaRobot) - 1) + Math.sin(angleGlobal) * Math.sin(angleDeltaRobot)) * xDeltaRobot / angleDeltaRobot;
        }

        angleGlobal  = (wheelCircumference * (fLeftNEW)/encoderPerRevolution - wheelCircumference * (fRightNEW)/encoderPerRevolution) / 14 * 0.51428571428;

        fLeftOLD = fLeftNEW;
        fRightOLD = fRightNEW;
        bLeftOLD = bLeftNEW;
        bRightOLD = bRightNEW;
    }
}