package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_ARM_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_ARM_PARKED;
import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_HOOK_HOOKED;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIN_ANGLE_ERROR;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIN_TURN_PWR;
import static org.firstinspires.ftc.teamcode.RobotConstants.moreThanMin;
import static org.firstinspires.ftc.teamcode.RobotConstants.notNaN;

@Autonomous
public class JustParkFarGoRight extends LinearOpMode
{
    private Servo capstoneArm;
    private Servo capstoneHook;

    private DcMotor frontLeft, frontRight,                  //front drive motors
                    backLeft, backRight;                    //rear drive motors

    private DcMotor     rightEncoder, leftEncoder, rearEncoder; //encoders

    //localizer///////////////////////////////////
    private Odometry localization;
    private LocalizerThread localizerThread;
    private Thread localizationUpdater;

    @Override
    public void runOpMode() throws InterruptedException
    {
        capstoneArm = hardwareMap.servo.get("capstoneArm");
        capstoneHook = hardwareMap.servo.get("capstoneHook");

        //locate drive motors from REV hub setup//
        frontLeft   = hardwareMap.get(DcMotor.class, "frontLeft");  //front left wheel
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight"); //front right wheel
        backLeft    = hardwareMap.get(DcMotor.class, "backLeft");   //back left wheel
        backRight   = hardwareMap.get(DcMotor.class, "backRight");  //back right wheel

        rearEncoder     = hardwareMap.get(DcMotor.class, "rearEncoder");
        rightEncoder    = hardwareMap.get(DcMotor.class, "intakeRight");
        leftEncoder     = hardwareMap.get(DcMotor.class, "intakeLeft");

        //start the position calculation system
        localization = new Odometry(rightEncoder, leftEncoder, rearEncoder);
        localizerThread = new LocalizerThread(localization);
        localizationUpdater = new Thread (localizerThread);
        
        capstoneArm.setPosition(CAPSTONE_ARM_IN);
        capstoneHook.setPosition(CAPSTONE_HOOK_HOOKED);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        localizationUpdater.start();

        capstoneArm.setPosition(CAPSTONE_ARM_PARKED);

        drive(51,0,0,false);

        localizerThread.stop();
    }

    private void drive (double x, double y, double angle, boolean foundation)
    {
        double  driveInput, strafeInput, turnInput,
                frontLeftPower, frontRightPower, backLeftPower, backRightPower;

        boolean isTurning = (angle != 0); //robot turns if angle is not 0

        x*=10;
        y*=10;
        angle = Math.toRadians(angle);

        double xStartPosition = localization.getxPosition();
        double yStartPosition = localization.getyPosition();
        double startAngle = localization.getAngle();

        double xTargetPosition = xStartPosition + x;
        double yTargetPosition = yStartPosition + y;
        double targetAngle = startAngle + angle;

        double dxTotal = xTargetPosition - xStartPosition;
        double dyTotal = yTargetPosition - yStartPosition;
        double dATotal = targetAngle - startAngle;

        double dx ,dy, dA;

        double xError, yError, angleError;

        boolean targetAcheived = false;

        double minPower = foundation? 0.3 : MIN_TURN_PWR;

        while (opModeIsActive() && !targetAcheived)
        {
            dx = xTargetPosition - localization.getxPosition();
            dy = yTargetPosition - localization.getyPosition();
            dA = targetAngle - localization.getAngle();

            xError = notNaN((dx)/dxTotal);
            yError = notNaN((dy)/dyTotal);
            angleError = notNaN((dA)/ (isTurning? Math.abs(dATotal) : Math.PI/8));

            //
            driveInput = y > 0 ? yError : -yError;//*cosine(Math.toRadians(angle)) + xError*sine(Math.toRadians(angle));
            turnInput  = 0.7*angleError;
            strafeInput = 0.75*(x > 0 ? xError : -xError);//*cosine(Math.toRadians(angle)) + yError*sine(Math.toRadians(angle));

            frontLeftPower = moreThanMin(driveInput + turnInput + strafeInput, minPower);
            frontRightPower = moreThanMin(driveInput - turnInput - strafeInput, minPower);
            backLeftPower = moreThanMin(driveInput + turnInput - strafeInput, minPower);
            backRightPower = moreThanMin(driveInput - turnInput + strafeInput, minPower);

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
            backLeft.setPower(backLeftPower);

            telemetry.addData("driveInput", driveInput);
            telemetry.addData("turnInput", turnInput);
            telemetry.addData("strafeInput", strafeInput);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("xTargetPosition", xTargetPosition);
            telemetry.addData("yTargetPosition", yTargetPosition);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("xPosition", localization.getxPosition());
            telemetry.addData("yPosition", localization.getyPosition());
            telemetry.addData("angle", Math.toDegrees(localization.getAngle()));
            telemetry.addData("angleError", angleError);
            telemetry.addData("dA", dA);
            telemetry.addData("dx", dx);
            telemetry.addData("dy", dy);
            telemetry.update();



            if ((!isTurning && dx > -20 && dx < 20 && dy > -20 && dy < 20) || (isTurning && Math.abs(angleError) < MIN_ANGLE_ERROR))
            {
                targetAcheived = true;
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
            }
        }
    }
}
