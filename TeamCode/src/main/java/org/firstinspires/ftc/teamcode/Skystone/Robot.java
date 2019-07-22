package org.firstinspires.ftc.teamcode.Skystone;


import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Robot {
    //Drive Motors
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;

    //positions
    public double xPos;
    public double yPos;
    public double anglePos;

    //imu
    private BNO055IMU imu;
    private Orientation angles;
    private Position position;

    //Inherited classes from Op Mode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode linearOpMode;

    //dimensions
    public double wheelRadius = 2;
    public final double wheelCircumference = 4 * Math.PI;
    public final double encoderPerRevolution = 806.4;
    public final double l = 7;
    public final double w = 6.5;

    //PID (concept only)



    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        //config names need to match configs on the phone
        //Map drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        //Set direction of drive motors
        fLeft.setDirection(DcMotor.Direction.FORWARD);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void intializeIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
    }

    public void resetEncoders(){
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void changeRunModeToUsingEncoder(){
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorMode(DcMotor.RunMode runMode){
        fLeft.setMode(runMode);
        fRight.setMode(runMode);
        bLeft.setMode(runMode);
        bRight.setMode(runMode);
    }

    //normal use method default 10 second kill time
    public void finalTurn(double targetHeading){
        finalTurn(targetHeading, 2000);
    }

    public void finalTurn(double targetHeading, long timeInMilli){
        targetHeading = Range.clip(targetHeading, -179, 179);

        long startTime = SystemClock.elapsedRealtime();

        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        position = imu.getPosition();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;
        double maxAngle = startHeading - targetHeading;
        maxAngle = Math.abs(maxAngle);

        int sign = 0;
        if(targetHeading > startHeading){
            sign = 1;
        }else{
            sign = -1;
        }
        if(maxAngle == 0){
            return;
        }
        while(linearOpMode.opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentDeltatAngle = Math.abs(angles.firstAngle - startHeading);
            double scaleFactor = currentDeltatAngle / maxAngle;
            double absolutePower = 1-scaleFactor;

            if(absolutePower< 0.01){
                absolutePower = 0.01;
            }
            double power = absolutePower * sign;
            if(scaleFactor > 1 || ((SystemClock.elapsedRealtime() - startTime) > timeInMilli)){
                break;
            }
            fLeft.setPower(-power);
            fRight.setPower(power);
            bLeft.setPower(-power);
            bRight.setPower(power);
        }
        brakeRobot();
        linearOpMode.sleep(100);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void allWheelDrive(double fLpower,double fRpower, double bLpower, double bRpower) {
        fLeft.setPower(fLpower);
        fRight.setPower(fRpower);
        bLeft.setPower(bLpower);
        bRight.setPower(bRpower);
    }

    public void resetMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void finalMove(double speed, double targetDistance) {
        //move robot function
        //to move backwards make targetDistance negative
        double rotations = 0;
        if (targetDistance>0) {
            rotations = targetDistance / 0.0168;
        } else {
            rotations = targetDistance / 0.0156;
        }
        moveRobot(speed,(int)(rotations));

        brakeRobot();
        linearOpMode.sleep(100);
    }

    public void moveRobot(double speed, int targetPostition){
        //called by final move - bare bones move function
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        double newSpeed = speed;

        if(targetPostition<0){
            newSpeed = newSpeed * -1;
            fLeft.setPower(newSpeed);
            fRight.setPower(newSpeed);
            bLeft.setPower(newSpeed);
            bRight.setPower(newSpeed);
        }else{
            fLeft.setPower(newSpeed);
            fRight.setPower(newSpeed);
            bLeft.setPower(newSpeed);
            bRight.setPower(newSpeed);
        }

        fLeft.setTargetPosition(targetPostition);
        fRight.setTargetPosition(targetPostition);
        bLeft.setTargetPosition(targetPostition);
        bRight.setTargetPosition(targetPostition);

        while(fLeft.isBusy() && fRight.isBusy() && bLeft.isBusy() && bRight.isBusy()
                && linearOpMode.opModeIsActive()){
        }
        brakeRobot();

        telemetry.addLine("finished sleeping");
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveMotorsBreakZeroBehavior() {
        //sets drive motors to brake mode
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void brakeRobot() {
        //brakes robot
        driveMotorsBreakZeroBehavior();
        fLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);
        bLeft.setPower(0);
    }

    public void wallFollow(double speed, double distance){
        //experimental - don't use
        //follows wall
        boolean notTimeLimit = true;
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLeft.setPower(speed);
        fRight.setPower(speed);
        bLeft.setPower(speed);
        bRight.setPower(speed);

        fLeft.setTargetPosition((int)(distance/0.028));
        fRight.setTargetPosition((int)(distance/0.028));
        bLeft.setTargetPosition((int)(distance/0.028));
        bRight.setTargetPosition((int)(distance/0.028));

        while(fLeft.isBusy() && linearOpMode.opModeIsActive()){
//            double leftPower = speed+normalizedDelta(frontRightDistance.getDistance(MM),70)/1000;
//            double rightPower = speed+normalizedDelta(backRightDistance.getDistance(MM),70)/1000;
//
//            fLeft.setPower(leftPower);
//            bLeft.setPower(leftPower);
//            fRight.setPower(rightPower);
//            bRight.setPower(rightPower);
//
//            telemetry.addData("frontRightDistance",frontRightDistance.getDistance(MM));
//            telemetry.addData("backRightDistance",backRightDistance.getDistance(MM));
//            telemetry.update();
//            if(frontDistance.getDistance(MM)<200 || frontFacingLeft.getDistance(MM)<200){
//                fLeft.setPower(0);
//                fRight.setPower(0);
//                bLeft.setPower(0);
//                bRight.setPower(0);
//                long startTime = SystemClock.elapsedRealtime();
//                while (((SystemClock.elapsedRealtime() - startTime) < 3000)){
//                    if(frontDistance.getDistance(MM)>200 && frontFacingLeft.getDistance(MM)>200){
//                        notTimeLimit = false;
//                        break;
//                    }else{
//                        notTimeLimit = true;
//                    }
//                }
//                if(notTimeLimit){
//                    finalTurn(0);
//                    wallFollow(0.7,-fLeft.getCurrentPosition()*0.028);
//                    brakeRobot();
//                }
//            }
        }
        brakeRobot();
    }

    private double normalizedDelta(double distance, double constant){
        //called in wallFollow
        double delta = distance-constant;
        if(delta>-50 && delta<50){
            delta = 0;
        }
        return delta;
    }

    public void setBrakeModeDriveMotors(){
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void splineMove(double[][] data) {

        resetEncoders();
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        long startTime = SystemClock.elapsedRealtime();
        double refLeftSpeed, refRightSpeed;
        double refLeftDistance, refRightDistance;
        double refHeading;
        double leftPower, rightPower;
        double leftDistance, rightDistance;
        double heading;
        int inc;
        int i;
        double encoderToInches = 0.156;  //515 encoders = 8 inches
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;

        /*double maxSpeed = 0;
        for (i=0; i<data.length; i++){
            if (maxSpeed < data[i][0]){
                maxSpeed = data[i][0];
            }
            if (maxSpeed < data[i][1]){
                maxSpeed = data[i][1];
            }
        }
        */


        while (linearOpMode.opModeIsActive()){

            double dt = SystemClock.elapsedRealtime() - startTime; //in milli
            dt = dt / 1000; //in seconds

            if (dt < data[data.length - 1][2]){                //find increment at any time to do interpolation
                inc = -1;
                for (i=0; i<data.length - 2; i++){
                    if (data[i][2] <= dt && dt < data[i+1][2]){
                        inc = i;
                        break;
                    }
                }
                if (inc < 0){
                    brakeRobot();
                    break;
                }
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                // find the left and right speed by interpolation from data file
                refLeftSpeed  = ((data[inc+1][0] - data[inc][0]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][0];
                refRightSpeed = ((data[inc+1][1] - data[inc][1]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][1];

                // find the left and right distance by interpolation from data file
                refLeftDistance = ((data[inc+1][4] - data[inc][4]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][5];
                refRightDistance = ((data[inc+1][5] - data[inc][5]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][5];

                // find the heading by interpolation from data file
                refHeading =((data[inc+1][6] - data[inc][6]) / (data[inc+1][2] - data[inc][2])) * (dt - data[inc][2]) + data[inc][6] + startHeading;

                // find the left and right encoder values and convert them to distance traveled in inches
                leftDistance = fLeft.getCurrentPosition() * encoderToInches;
                rightDistance = fRight.getCurrentPosition() * encoderToInches;

                // find the heading of robot
                heading = angles.firstAngle;

                // find power
                leftPower  = refLeftSpeed;
                rightPower = refRightSpeed;

                // set power
                fLeft.setPower(leftPower);
                bLeft.setPower(leftPower);
                fRight.setPower(rightPower);
                bRight.setPower(rightPower);
            } else {
                brakeRobot();
                break;
            }
        }

    }

//    public void odometryUsingCircles() {
//        double fLeftNEW = fLeft.getCurrentPosition();
//        double fRightNEW = fRight.getCurrentPosition();
//        double bLeftNEW = bLeft.getCurrentPosition();
//        double bRightNEW = bRight.getCurrentPosition();
//        double r;
//
//        double deltaLeft = wheelCircumference * (fLeftNEW-fLeftOLD)/encoderPerRevolution;
//        double deltaRight = wheelCircumference * (fRightNEW-fRightOLD)/encoderPerRevolution;
//
//        if (deltaRight == deltaLeft){
//            yDeltaRobot = deltaRight;
//        } else {
//            r = l * (deltaRight / (deltaLeft-deltaRight) + 1/2);
//            angleDeltaRobot = (deltaLeft-deltaRight)/14 * 0.51428571428;
//            xDeltaRobot = r * (1 - Math.cos(angleDeltaRobot));
//            yDeltaRobot = r * Math.sin(angleDeltaRobot);
//        }
//
//        //converting to global frame
//        xPosGlobal += xDeltaRobot * Math.cos(angleGlobal) - yDeltaRobot * Math.sin(angleGlobal);
//        yPosGlobal += xDeltaRobot * Math.sin(angleGlobal) + yDeltaRobot * Math.cos(angleGlobal);
//        angleGlobal  = (wheelCircumference * (fLeftNEW)/encoderPerRevolution - wheelCircumference * (fRightNEW)/encoderPerRevolution) / 14 * 0.51428571428;
//
//        fLeftOLD = fLeftNEW;
//        fRightOLD = fRightNEW;
//        bLeftOLD = bLeftNEW;
//        bRightOLD = bRightNEW;
//    }

    //OPTIMAL ANGLE WILL BE 90 ON MOST CASES BECAUSE THE OPTIMAL ANGLE IS FORWARD
    public void goToPoint(double x, double y, double speedRatio, double turnSpeed, double optimalAngle){

        double xPos = this.xPos;
        double yPos = this.yPos;
        double anglePos = this.anglePos;

        while(linearOpMode.opModeIsActive()){
            double distanceToTarget = Math.hypot(x-xPos, y-yPos);
            double absoluteAngleToTarget = Math.atan2(y - yPos, x - xPos);

            double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - (Math.toRadians(anglePos))-Math.toRadians(90));
            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
            double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + Math.toRadians(90);

            double xPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double yPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));

            double xMovement = xPower * speedRatio;
            double yMovement = yPower * speedRatio;
            double turnMovement = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

            if(distanceToTarget < 10) { turnMovement = 0; }

            //the signs need tweaking
            double fLeftPower = (-yMovement-turnMovement+xMovement*1.414);
            double bLeftPower = (-yMovement-turnMovement-xMovement*1.414);
            double bRightPower = (-yMovement-turnMovement-xMovement*1.414);
            double fRightPower = (-yMovement-turnMovement+xMovement*1.414);

            // op scaling
            // try this later: use true maximum speed by finding the largest of each motor's powers and generalizing that to 1
//            double maxRawPower = Math.abs(fLeftPower);
//            if(Math.abs(bLeftPower) > maxRawPower){ maxRawPower = Math.abs(bLeftPower);}
//            if(Math.abs(bRightPower) > maxRawPower){ maxRawPower = Math.abs(bRightPower);}
//            if(Math.abs(fRightPower) > maxRawPower){ maxRawPower = Math.abs(fRightPower);}
//
//            fLeftPower *= speedRatio;
//            fRightPower *= speedRatio;
//            bLeftPower *= speedRatio;
//            bRightPower *= speedRatio;

            fLeft.setPower(fLeftPower);
            fRight.setPower(fRightPower);
            bLeft.setPower(bLeftPower);
            bRight.setPower(bRightPower);
        }
    }
}