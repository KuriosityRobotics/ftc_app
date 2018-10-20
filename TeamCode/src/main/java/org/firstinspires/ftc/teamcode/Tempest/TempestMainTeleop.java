package org.firstinspires.ftc.teamcode.Tempest;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DigitalChannel;
        import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Temepst: Main Teleop", group="Linear Opmode")
//@Disabled
public class TempestMainTeleop extends LinearOpMode
{
    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;
    long startTime = 0;
    boolean changedRight = false, onRight = false;
    boolean changedLeft = false, onLeft = false;

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode(){
        //Init's robot
        Tempest robot = new Tempest(hardwareMap, telemetry, this);   //DO NOT DELETE
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Clears power values
        fLPower = 0.0;
        fRPower = 0.0;
        bLPower = 0.0;
        bRPower = 0.0;
        double powerScaleFactor = 0.4;
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

            if(gamepad1.y){
                robot.pivot.setPower(-1);
            }else if(gamepad1.x){
                robot.lock.setPosition(0.5);
                robot.pivot.setPower(1);
            }else{
                robot.pivot.setPower(0);
            }
            if(gamepad1.a){
                robot.lock.setPosition(0);
            }

            if(robot.upTouch.isPressed()){
                robot.lock.setPosition(1);
                robot.pivot.setPower(0);
            }
            if(gamepad2.x && gamepad2.right_bumper) {
                robot.intakeLeft.setPosition(0.5);
            }else if(gamepad2.x){
                robot.intakeLeft.setPosition(0.7);
            }
            if(gamepad2.b && gamepad2.right_bumper) {
                robot.intakeRight.setPosition(0.35);
            }else if(gamepad2.b){
                robot.intakeRight.setPosition(0.2);
            }
            if(gamepad2.y&&gamepad2.right_bumper){
                robot.intakeRight.setPosition(0.35);
                robot.intakeLeft.setPosition(0.5);
            }else if(gamepad2.y){
                robot.intakeLeft.setPosition(0.7);
                robot.intakeRight.setPosition(0.2);
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