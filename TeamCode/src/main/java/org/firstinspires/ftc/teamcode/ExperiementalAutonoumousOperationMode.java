package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

@Autonomous(name="ExpAuto", group="Linear OpMode")
@Disabled
public class ExperiementalAutonoumousOperationMode extends LinearOpMode
{
    private DcMotor frontLeft = null;                   //front left motor
    private DcMotor frontRight = null;                  //front right motor
    private DcMotor backLeft = null;                    //back left motor
    private DcMotor backRight = null;                   //back right motor

    private DcMotor lift = null;                        //lift motor

    private Servo leftFoundGrabber = null;              //left foundation grabber
    private Servo rightFoundGrabber = null;             //right foundation grabber

    private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private DcMotor rearEncoder;

    private Odometry localization;
    private LocalizerThread localizerThread;
    private Thread localizationUpdater;

    private double driveInput   = 0.0;
    private double strafeInput  = 0.0;
    private double turnInput    = 0.0;

    private double frontLeftPower   = 0.0;
    private double frontRightPower  = 0.0;
    private double backLeftPower    = 0.0;
    private double backRightPower   = 0.0;

    public void runOpMode()
    {
        //locate drive motors from REV hub setup//
        frontLeft   = hardwareMap.get(DcMotor.class, "frontLeft");  //front left wheel
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight"); //front right wheel
        backLeft    = hardwareMap.get(DcMotor.class, "backLeft");   //back left wheel
        backRight   = hardwareMap.get(DcMotor.class, "backRight");  //back right wheel

        rightEncoder = hardwareMap.get(DcMotor.class, "intakeRight");
        leftEncoder = hardwareMap.get(DcMotor.class, "intakeLeft");
        rearEncoder = hardwareMap.get(DcMotor.class, "rearEncoder");

        lift        = hardwareMap.get(DcMotor.class, "lift");

        leftFoundGrabber    = hardwareMap.servo.get("foundGrabberLeft");
        rightFoundGrabber   = hardwareMap.servo.get("foundGrabberRight");

        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        rightEncoder.setDirection(DcMotor.Direction.REVERSE);
        leftEncoder.setDirection(DcMotor.Direction.FORWARD);

        //reset and initialize lift motor//
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        localization = new Odometry(rightEncoder, leftEncoder, rearEncoder);
        localizerThread = new LocalizerThread(localization);
        localizationUpdater = new Thread (localizerThread);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        Thread setFoundationGrabber = new Thread (new FoundationGrabberSet(lift, leftFoundGrabber, rightFoundGrabber));
        setFoundationGrabber.start();

        localizationUpdater.start();

        sleep(2500);

        drive (50, 50, 0, false, 1);

        localizerThread.stop();
    }

    private void drive (double x, double y, double angle, boolean foundation, double power)
    {
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
            localization.update();

            dx = xTargetPosition - localization.getxPosition();
            dy = yTargetPosition - localization.getyPosition();
            dA = targetAngle - localization.getAngle();

            xError      = notNaN((dx)/ ((isTurning || x == 0)? 200 : Math.abs(dxTotal)));
            yError      = notNaN((dy)/ ((isTurning || y == 0)? 200 : Math.abs(dyTotal)));
            angleError  = notNaN((dA)/ (isTurning? Math.abs(dATotal) : Math.PI/6));

            driveInput = yError*cosine(Math.toRadians(angle)) + xError*sine(Math.toRadians(angle));
            strafeInput = xError*cosine(Math.toRadians(angle)) - yError*sine(Math.toRadians(angle));
            turnInput  = 0.7*angleError;

            //full speed while not slow mode
            frontLeftPower = power*moreThanMin(driveInput + turnInput + strafeInput, minPower);
            frontRightPower = power*moreThanMin(driveInput - turnInput - strafeInput, minPower);
            backLeftPower = power*moreThanMin(driveInput + turnInput - strafeInput, minPower);
            backRightPower = power*moreThanMin(driveInput - turnInput + strafeInput, minPower);

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

    public void driveWithoutAccuracy (double distanceCM, double power)
    {
        double startPosition = frontLeft.getCurrentPosition();
        double targetPosition = startPosition + distanceCM*TICKS_PER_CM;
        if (distanceCM > 0)
        {
            while (opModeIsActive() && frontLeft.getCurrentPosition() < targetPosition) {
                frontRight.setPower(power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(power);
            }
        }
        else
        {
            while (opModeIsActive() && frontLeft.getCurrentPosition() > targetPosition)
            {
                frontRight.setPower(-power);
                frontLeft.setPower(-power);
                backRight.setPower(-power);
                backLeft.setPower(-power);
                telemetry.addData("Front Left Position: ", frontLeft.getCurrentPosition());
                telemetry.update();
            }
        }

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
}

