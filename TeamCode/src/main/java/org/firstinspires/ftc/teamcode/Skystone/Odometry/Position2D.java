package org.firstinspires.ftc.teamcode.Skystone.Odometry;

import android.app.ProgressDialog;
import android.os.AsyncTask;
import android.os.Looper;
import android.support.annotation.MainThread;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Skystone.MathFunctions;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class Position2D{
    FtcRobotControllerActivity activity;
    Robot robot;
    public Position2D(Robot robot) {
        this.robot = robot;
    }

    public void startOdometry(){
        Odometry o = new Odometry(robot);
        LongRunningTask longRunningTask = new LongRunningTask(robot,o);
        longRunningTask.execute();
    }
}
class LongRunningTask extends AsyncTask<Void, Boolean, Boolean> {
    Robot robot;
    Odometry o;
    public LongRunningTask(Robot robot, Odometry o){
        this.robot = robot;
        this.o = o;
    }

    @Override
    protected Boolean doInBackground(Void... params) {
        while(robot.linearOpMode.opModeIsActive()) {
            o.constantVelocityOdometry();
            robot.xPos = o.xPosGlobal;
            robot.yPos = o.yPosGlobal;
            robot.anglePos = o.angleGlobal;
        }
        return true;
    }

    protected void onPostExecute(Boolean result) {
        if(result) {
            robot.telemetry.addLine("DONE");
            robot.telemetry.update();
        }
    }

}

class Odometry{

    public Robot robot;

    double xPosGlobal = 0;
    double yPosGlobal = 0;
    double angleGlobal = 0;

    public double angleDeltaRobot;
    public double xDeltaRobot;
    public double yDeltaRobot;

    private double fLeftNEW = 0;
    private double fRightNEW = 0;
    private double bLeftNEW = 0;
    private double bRightNEW = 0;

    private double fLeftOLD = 0;
    private double fRightOLD = 0;
    private double bLeftOLD = 0;
    private double bRightOLD = 0;

    private double fl;
    private double fr;
    private double bl;
    private double br;

    public Odometry(Robot robot){
        this.robot = robot;
    }
    public void constantVelocityOdometry() {

        fLeftNEW = robot.fLeft.getCurrentPosition();
        fRightNEW = robot.fRight.getCurrentPosition();
        bLeftNEW = robot.bLeft.getCurrentPosition();
        bRightNEW = robot.bRight.getCurrentPosition();

        // find robot position
        fl = 2 * Math.PI * (fLeftNEW - fLeftOLD) / robot.encoderPerRevolution;
        fr = 2 * Math.PI * (fRightNEW - fRightOLD) / robot.encoderPerRevolution;
        bl = 2 * Math.PI * (bLeftNEW - bLeftOLD) / robot.encoderPerRevolution;
        br = 2 * Math.PI * (bRightNEW - bRightOLD) / robot.encoderPerRevolution;

        xDeltaRobot = robot.wheelRadius/4 * (fl + bl + br + fr);
        yDeltaRobot = robot.wheelRadius/4 * (-fl + bl - br + fr);
        angleDeltaRobot = robot.wheelRadius/4 *(-fl/(robot.l+robot.w) - bl/(robot.l+robot.w) + br/(robot.l+robot.w) + fr/(robot.l+robot.w));

        //converting to global frame
        if (angleDeltaRobot == 0){
            xPosGlobal += xDeltaRobot * Math.cos(angleGlobal) - yDeltaRobot * Math.sin(angleGlobal);
            yPosGlobal += xDeltaRobot * Math.sin(angleGlobal) + yDeltaRobot * Math.cos(angleGlobal);
        } else {
            xPosGlobal += (Math.cos(angleGlobal) * Math.sin(angleDeltaRobot) - (Math.cos(angleDeltaRobot) - 1) * Math.sin(angleGlobal)) * xDeltaRobot / angleDeltaRobot + (Math.cos(angleGlobal) * (Math.cos(angleDeltaRobot) - 1) - Math.sin(angleGlobal) * Math.sin(angleDeltaRobot)) * yDeltaRobot / angleDeltaRobot;
            yPosGlobal += ((Math.cos(angleDeltaRobot) - 1) * Math.sin(angleGlobal) + (Math.cos(angleGlobal)) * Math.sin(angleDeltaRobot)) * yDeltaRobot / angleDeltaRobot + (Math.cos(angleGlobal) * (Math.cos(angleDeltaRobot) - 1) + Math.sin(angleGlobal) * Math.sin(angleDeltaRobot)) * xDeltaRobot / angleDeltaRobot;
        }

        angleGlobal = MathFunctions.AngleWrap((robot.wheelCircumference * (fLeftNEW)/robot.encoderPerRevolution - robot.wheelCircumference * (fRightNEW)/robot.encoderPerRevolution) / 14 * 0.51428571428);

        fLeftOLD = fLeftNEW;
        fRightOLD = fRightNEW;
        bLeftOLD = bLeftNEW;
        bRightOLD = bRightNEW;

        robot.telemetry.addLine("XPOS: " + xPosGlobal);
        robot.telemetry.addLine("YPOS: " + yPosGlobal);
        robot.telemetry.addLine("ANGPOS: " + Math.toDegrees(Math.toDegrees(angleGlobal)));
        robot.telemetry.update();
    }
}