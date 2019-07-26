package org.firstinspires.ftc.teamcode.Skystone.MotionProfiler;

import java.util.Vector;

public class PathPoints {
    public Vector<CurvePoint> targetPoints = new Vector<>();
    final double moveSpeed = 1;
    final double turnSpeed = 1;
    final double followDistance = 4;
    final double followRadius = Math.toRadians(50);
    final double slowDownTurnAmount = 1;

    public PathPoints(double[][] points){
        for(int i = 0;i<points.length;i++){
            targetPoints.add(new CurvePoint(points[i][0],points[i][1],moveSpeed,turnSpeed,followDistance,followRadius,slowDownTurnAmount));
        }
    }

    public PathPoints(double[][] points, double moveSpeed, double turnSpeed, double followDistance, double followRadius, double slowDownTurnAmount) {
        for (int i = 0; i < points.length; i++) {
            targetPoints.add(new CurvePoint(points[i][0], points[i][1], moveSpeed, turnSpeed, followDistance, Math.toRadians(followRadius), slowDownTurnAmount));
        }
    }

    public void setNewPoints(double[][] points){
        targetPoints.clear();
        for(int i = 0;i<points.length;i++){
            targetPoints.add(new CurvePoint(points[i][0],points[i][1],moveSpeed,turnSpeed,followDistance,followRadius,slowDownTurnAmount));
        }
    }

    public void setNewPoints(double[][] points, double moveSpeed, double turnSpeed, double followDistance, double followRadius, double slowDownTurnAmount){
        targetPoints.clear();
        for(int i = 0;i<points.length;i++){
            targetPoints.add(new CurvePoint(points[i][0], points[i][1], moveSpeed, turnSpeed, followDistance, Math.toRadians(followRadius), slowDownTurnAmount));
        }
    }
}
