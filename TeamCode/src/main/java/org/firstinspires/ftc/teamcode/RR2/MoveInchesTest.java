package org.firstinspires.ftc.teamcode.RR2;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="MoveInches test", group="Linear Opmode") //name of your program on the phone and defines if it is teleop or auto
//@Disabled
public class MoveInchesTest extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
        //Init's robot
        RR2 robot = new RR2(hardwareMap,telemetry,this);
        robot.driveMotorsBreakZeroBehavior();
        waitForStart();
        runtime.reset();
        robot.intializeIMU();
        while (opModeIsActive()){
//            robot.intializeIMU();
//            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            double startAngle = robot.angles.thirdAngle;
//            robot.fLeft.setPower(0.1);
//            robot.bLeft.setPower(0.1);
//            telemetry.addLine("" + (robot.angles.thirdAngle));
            //robot.moveRobotInches(0.5, 20);
            robot.straightLine(0.2, 20);
            robot.brakeRobot();
            sleep(100000000);
        }
        //Run1: 16.5  Run3:17 Run4: 18.25 Run5: 19.25
    }
}