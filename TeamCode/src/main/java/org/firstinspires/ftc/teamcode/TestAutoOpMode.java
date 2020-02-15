package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

@Autonomous(name="Blue Close", group="Linear OpMode")
public class TestAutoOpMode extends LinearOpMode
{
    private DcMotor frontLeft = null;                   //front left motor
    private DcMotor frontRight = null;                  //front right motor
    private DcMotor backLeft = null;                    //back left motor
    private DcMotor backRight = null;                   //back right motor
    private ModernRoboticsI2cColorSensor colorSensor = null;    //color sensor

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

        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");

        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
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

        //go to the foundation
        drive (12, -77, 0, true);

        //grab the foundation
        leftFoundGrabber.setPosition(LEFT_FOUND_DOWN);
        rightFoundGrabber.setPosition(RIGHT_FOUND_DOWN);

        //wait until the grabbers have grabbed
        sleep(750);

        //turn the foundation
        drive (0, 0, -90, true);

        //push into wall
        driveWithoutAccuracy (-80, 1);

        //let go of foundation
        leftFoundGrabber.setPosition(LEFT_FOUND_UP);
        rightFoundGrabber.setPosition(RIGHT_FOUND_UP);

        //reset localization angle
        localization.resetAngle();

        //drive away from the foundation
        drive (62, 80, 0, false);

        //go to the line with sensor
        driveUntilLine("blue", 0.4);

        //bump into the wall to reorient
        strafe(45, true, 0.8);

        localizerThread.stop();
    }

    private void drive (double x, double y, double angle, boolean foundation)
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
            dx = xTargetPosition - localization.getxPosition();
            dy = yTargetPosition - localization.getyPosition();
            dA = targetAngle - localization.getAngle();

            xError = notNaN((dx)/dxTotal);
            yError = notNaN((dy)/dyTotal);
            angleError = notNaN((dA)/ (isTurning? Math.abs(dATotal) : Math.PI/2));

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

    public void driveUntilLine (String lineColor, double power)
    {
        if (lineColor.equals("blue"))
        {
            while (opModeIsActive() && colorSensor.blue() < 9)
            {
                frontRight.setPower(power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(power);
            }
        }
        else if (lineColor.equals("red"))
        {
            while (opModeIsActive() && colorSensor.red() < 9)
            {
                frontRight.setPower(power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(power);
            }
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void strafe (double distanceCM, boolean right, double power)
    {
        double currentPosition = frontLeft.getCurrentPosition();
        double targetPosition;

        if (right)
        {
            targetPosition = currentPosition + distanceCM*TICKS_PER_CM;

            while (opModeIsActive() && frontLeft.getCurrentPosition() < targetPosition)
            {
                frontRight.setPower(-power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(-power);
                telemetry.addData("Front Left Position: ", frontLeft.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            targetPosition = currentPosition - distanceCM*TICKS_PER_CM;

            while (opModeIsActive() && frontLeft.getCurrentPosition() > targetPosition)
            {
                frontRight.setPower(power);
                frontLeft.setPower(-power);
                backRight.setPower(-power);
                backLeft.setPower(power);
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

