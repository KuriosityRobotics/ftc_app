package org.firstinspires.ftc.teamcode.Tempest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Temepst: Main Teleop", group="Linear Opmode")
//@Disabled
public class TempestMainTeleop extends LinearOpMode
{

    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

    public static double powerScaleFactor = 0.4;

    long startTime = 0;
    boolean changedRight = false, onRight = false;
    boolean changedLeft = false, onLeft = false;
    boolean changedBothLeft = false, onDoubleLeft = false;
    boolean changedBothRight = false, onDoubleRight = false;
    boolean changedLocks = false, onBoth = false;




    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
        //Init's robot
        Tempest robot = new Tempest(hardwareMap, telemetry, this);   //DO NOT DELETE
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Clears power values
        fLPower = 0.0;
        fRPower = 0.0;
        bLPower = 0.0;
        bRPower = 0.0;
        double powerScaleFactor = 0.6;
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            //tank drive
            fLPower = -(gamepad1.left_stick_y)*powerScaleFactor;
            bLPower = -(gamepad1.left_stick_y)*powerScaleFactor;
            fRPower = -(gamepad1.right_stick_y)*powerScaleFactor;
            bRPower = -(gamepad1.right_stick_y)*powerScaleFactor;
            //Straight D-Pad move
            if (gamepad1.dpad_up) {
                fLPower = (gamepad1.left_stick_y)+powerScaleFactor;
                bLPower = (gamepad1.left_stick_y)+powerScaleFactor;
                fRPower = (gamepad1.right_stick_y+powerScaleFactor);
                bRPower = (gamepad1.right_stick_y+powerScaleFactor);
            } else if (gamepad1.dpad_down) {
                fLPower = (gamepad1.left_stick_y)-powerScaleFactor;
                bLPower = (gamepad1.left_stick_y)-powerScaleFactor;
                fRPower = (gamepad1.right_stick_y-powerScaleFactor);
                bRPower = (gamepad1.right_stick_y)-powerScaleFactor;
            } else if (gamepad1.dpad_right) {
                fLPower = (gamepad1.right_stick_y+powerScaleFactor);
                bLPower = (gamepad1.right_stick_y)+powerScaleFactor;
                fRPower = (gamepad1.left_stick_y)-powerScaleFactor;
                bRPower = (gamepad1.left_stick_y)-powerScaleFactor;
            } else if (gamepad1.dpad_left) {
                fRPower = (gamepad1.right_stick_y+powerScaleFactor);
                bRPower = (gamepad1.right_stick_y)+powerScaleFactor;
                fLPower = (gamepad1.left_stick_y)-powerScaleFactor;
                bLPower = (gamepad1.left_stick_y)-powerScaleFactor;
            }
            robot.fLeft.setPower(fLPower);
            robot.fRight.setPower(fRPower);
            robot.bLeft.setPower(bLPower);
            robot.bRight.setPower(bRPower);

            robot.slideRight.setPower(gamepad2.right_stick_y);
            robot.slideLeft.setPower(-gamepad2.right_stick_y);

            if(gamepad2.dpad_up){
                robot.pivot.setPower(-0.75);
            }else if(gamepad2.dpad_down){
                robot.lock.setPosition(0.25);
                robot.pivot.setPower(0.75);
            }else{
                robot.pivot.setPower(0);
            }

            if(gamepad1.left_bumper){
                robot.lock.setPosition(0.25);
            }else if(robot.upTouch.isPressed()){
                robot.lock.setPosition(0.75);
                robot.pivot.setPower(0);
            }
//            if(gamepad2.x && gamepad2.right_bumper) {
//                robot.intakeLeft.setPosition(0.5);
//            }else if(gamepad2.x){
//                //close
//                robot.intakeLeft.setPosition(0.7);
//            }
//
//            if(gamepad2.b && gamepad2.right_bumper) {
//                robot.intakeRight.setPosition(0.35);
//            }else if(gamepad2.b){
//                //close
//                robot.intakeRight.setPosition(0.2);
//            }

            if(gamepad2.x && !changedLeft){
//                robot.intakeLeft.setPosition(onLeft ? 0.5 : 0.7);
                robot.intakeLeft.setPower(onLeft ? -1 : 1);
                onLeft = !onLeft;
                changedLeft = true;
            }else if(!gamepad2.x){
                changedLeft = false;
            }

            if(gamepad2.a && !changedRight){
//                robot.intakeRight.setPosition(onRight ? 0.2 : 0.35);
                robot.intakeRight.setPower(onLeft ? -1 : 1);
                onRight = !onRight;
                changedRight = true;
            }else if(!gamepad2.a){
                changedRight = false;
            }

            robot.intakeRight.setPower(gamepad2.left_stick_y);
            robot.intakeLeft.setPower(-gamepad2.left_stick_y);

            if(gamepad1.left_bumper){
                robot.hook.setPosition(0.575);
            }else if(gamepad1.right_bumper){
                robot.hook.setPosition(0.2);
            }

            if(gamepad1.a){
                robot.hangLockLeft.setPosition(0.2);
                robot.hangLockRight.setPosition(0.15);
            }else if(gamepad1.b){
                robot.hangLockLeft.setPosition(0.8);
                robot.hangLockRight.setPosition(1);
            }

            if(gamepad1.x && gamepad1.right_bumper){
                robot.intake.setPower(0.3);
            }else if(gamepad1.y && gamepad1.right_bumper){
                robot.intake.setPower(-0.3);
            }else if(gamepad1.y){
                robot.intake.setPower(-0.5);
            }else if(gamepad1.x){
                robot.intake.setPower(0.65);
            }else{
                robot.intake.setPower(0);
            }




            if (robot.upTouch.isPressed()) {
                telemetry.addData("Digital Touch", "Is ressed");
            } else {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            }
            telemetry.update();
        }
    }
}