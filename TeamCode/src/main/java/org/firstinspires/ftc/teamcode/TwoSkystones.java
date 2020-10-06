package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLOCK_GRABBER_CLOSED;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLOCK_GRABBER_OPEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLOCK_LIFTER_DOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLOCK_LIFTER_LIFTED;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLOCK_LIFTER_UP;
import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_ARM_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_ARM_PARKED;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_ARM_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_ARM_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_HIGHEST_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_FOUND_DOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_FOUND_UP;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIN_ANGLE_ERROR;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIN_TURN_PWR;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_FOUND_DOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_FOUND_UP;
import static org.firstinspires.ftc.teamcode.RobotConstants.SkystonePosition;
import static org.firstinspires.ftc.teamcode.RobotConstants.SkystonePosition.RIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.TICKS_PER_CM;
import static org.firstinspires.ftc.teamcode.RobotConstants.moreThanMin;
import static org.firstinspires.ftc.teamcode.RobotConstants.notNaN;

@Autonomous
public class TwoSkystones extends LinearOpMode
{
    //Declare OpMode members////////////////////
    private ElapsedTime runtime;                                //runtime counter

    private DcMotor     frontLeft, frontRight,                  //front drive motors
                        backLeft, backRight;                    //rear drive motors
    private DcMotor     intakeLeft, intakeRight;                //intake wheel motors
    private DcMotor     lift;                                   //lift motor

    private DcMotor     rightEncoder, leftEncoder, rearEncoder; //encoders

    private Servo       claw, clawArm,                          //claw servos
                        capstoneArm, capstoneHook,              //capstone servos
                        blockLifter, blockGrabber,              //block grabber servos
                        leftFoundGrabber, rightFoundGrabber;    //foundation grabbers

    private ModernRoboticsI2cColorSensor colorSensor;           //color sensor

    //localizer///////////////////////////////////
    private Odometry localization;

    //Skystone Detection//////////////////////////
    private OpenCvCamera                webcam;
    private SkystoneDetectorPipeline    skystoneDetector;
    private SkystonePosition            skystonePosition;

    //initialization variables////////////////////
    private enum StartPosition {RED_BLOCKS, BLUE_BLOCKS, RED_BUILD, BLUE_BUILD}
    boolean farPark         = false;    //park far
    boolean moveFoundation  = false;    //move the foundation
    boolean getBlocks       = false;    //get the blocks

    private void executeAutonomous (StartPosition pos)
    {
        switch (pos)
        {
            case RED_BLOCKS: doRedBlock();
                break;
            case RED_BUILD: doRedBuild();
                break;
            case BLUE_BUILD: doBlueBuild();
                break;
            case BLUE_BLOCKS: doBlueBlocks();
                break;
        }
    }

    private void doRedBlock ()
    {
        //lift block
        blockLifter.setPosition(BLOCK_LIFTER_LIFTED);
        //go to blocks
        switch (skystonePosition)
        {
            case LEFT: drive(68, 17,true);
                break;
            case CENTER: drive (68, 0, true);
                break;
            case RIGHT: drive(68,-23,true);
                break;
            case NONE: drive(68, 17.0,true);
                break;
        }

        //grab block
        blockLifter.setPosition(BLOCK_LIFTER_DOWN);
        sleep(400);
        blockGrabber.setPosition(BLOCK_GRABBER_CLOSED);
        sleep(600); //changed from 250 milliseconds
        blockLifter.setPosition(BLOCK_LIFTER_UP);

        //drive away from blocks
        switch (skystonePosition)
        {
            case LEFT: drive(60, 0, false);
                break;
            case CENTER: drive(60, 0.0, false);
                break;
            case RIGHT: drive(60, 0.00, false);
                break;
            case NONE: drive(60, 00.0, false);
                break;
        }

        //TODO: Center and Right
        switch (skystonePosition)
        {
            case LEFT: drive(-110, -60);
                break;
            case CENTER: drive(-130, -80);
                break;
            case RIGHT: drive(-150, -100);
                break;
            case NONE: drive(-110.0, -60);
                break;
        }

        blockLifter.setPosition(BLOCK_LIFTER_LIFTED);
        blockGrabber.setPosition(BLOCK_GRABBER_OPEN);

        sleep(500);

        if (skystonePosition != RIGHT)
        {
            //TODO: Center and Right
            switch (skystonePosition)
            {
                case LEFT: drive(-175, -120);
                    break;
                case CENTER: drive(-185, -130);
                    break;
                case RIGHT: drive(-210, -160);
                    break;
                case NONE: drive(-175.0, -120);
                    break;
            }

            //drive to from blocks
            switch (skystonePosition)
            {
                case LEFT: drive(68, 0, false);
                    break;
                case CENTER: drive(68, 0.0, false);
                    break;
                case RIGHT: drive(68, 0.00, false);
                    break;
                case NONE: drive(68, 00.0, false);
                    break;
            }

            //grab block
            blockLifter.setPosition(BLOCK_LIFTER_DOWN);
            sleep(400);
            blockGrabber.setPosition(BLOCK_GRABBER_CLOSED);
            sleep(600); //changed from 250 milliseconds
            blockLifter.setPosition(BLOCK_LIFTER_UP);

            //drive away from blocks
            switch (skystonePosition)
            {
                case LEFT: drive(60, 0, false);
                    break;
                case CENTER: drive(60, 0.0, false);
                    break;
                case RIGHT: drive(60, 0.00, false);
                    break;
                case NONE: drive(60, 00.0, false);
                    break;
            }

            //TODO: Center and Right
            switch (skystonePosition)
            {
                case LEFT: drive(-175, -120);
                    break;
                case CENTER: drive(-195, -140);
                    break;
                case RIGHT: drive(-215, -160);
                    break;
                case NONE: drive(-170.0, -120);
                    break;
            }

            blockLifter.setPosition(BLOCK_LIFTER_LIFTED);
            blockGrabber.setPosition(BLOCK_GRABBER_OPEN);

            sleep(500);
        }
        else
        {
            /*
            drive(-190, -140);

            turn(90,false);
            turn(90, false);
            localization.resetPosition();

            drive(-20.0,0,false);

            intake(true);

            drive(0.,8,false);

            claw.setPosition(CLAW_CLOSED);

            drive(0., -8, false);

            drive (20.,0,false);

            drive(190, 140);

            while (lift.getCurrentPosition() < CLAW_HIGHEST_POSITION)
            {
                lift.setPower(0.5);
            }
            lift.setPower(0);
            clawArm.setPosition(CLAW_ARM_OUT);
            sleep(500);
            claw.setPosition(CLAW_OPEN);
            sleep(250);
            clawArm.setPosition(CLAW_ARM_IN);
            sleep(100);
            while (lift.getCurrentPosition() > CLAW_HIGHEST_POSITION)
            {
                lift.setPower(-.5);
            }

             */
        }

        blockLifter.setPosition(BLOCK_LIFTER_UP);

        driveUntilLine("blue", -0.5);
    }

    private void initialize()
    {
        claw.setPosition(CLAW_OPEN);
        clawArm.setPosition(CLAW_ARM_IN);
        leftFoundGrabber.setPosition(LEFT_FOUND_UP);
        capstoneArm.setPosition(CAPSTONE_ARM_IN);
        sleep(100);
        blockGrabber.setPosition(BLOCK_GRABBER_OPEN);
        blockLifter.setPosition(BLOCK_LIFTER_UP);
    }
    private void doRedBuild ()
    {

    }

    private void doBlueBuild ()
    {

    }

    private void doBlueBlocks ()
    {
        //lift block
        blockLifter.setPosition(BLOCK_LIFTER_LIFTED);
        //go to blocks
        switch (skystonePosition)
        {
            case LEFT: drive(68, 17,true);
                break;
            case CENTER: drive (68, 0, true);
                break;
            case RIGHT: drive(68,-23,true);
                break;
            case NONE: drive(68, 17.0,true);
                break;
        }

        //grab block
        blockLifter.setPosition(BLOCK_LIFTER_DOWN);
        sleep(400);
        blockGrabber.setPosition(BLOCK_GRABBER_CLOSED);
        sleep(600); //changed from 250 milliseconds
        blockLifter.setPosition(BLOCK_LIFTER_UP);

        //drive away from blocks
        switch (skystonePosition)
        {
            case LEFT: drive(60, 0, false);
                break;
            case CENTER: drive(60, 0.0, false);
                break;
            case RIGHT: drive(60, 0.00, false);
                break;
            case NONE: drive(60, 00.0, false);
                break;
        }

        //TODO: Center and Right
        switch (skystonePosition)
        {
            case LEFT: drive(110, 60);
                break;
            case CENTER: drive(130, 80);
                break;
            case RIGHT: drive(150, 100);
                break;
            case NONE: drive(110.0, 60);
                break;
        }

        blockLifter.setPosition(BLOCK_LIFTER_LIFTED);
        blockGrabber.setPosition(BLOCK_GRABBER_OPEN);

        sleep(500);

        if (skystonePosition != RIGHT)
        {
            //TODO: Center and Right
            switch (skystonePosition)
            {
                case LEFT: drive(-175, -120);
                    break;
                case CENTER: drive(-185, -130);
                    break;
                case RIGHT: drive(-210, -160);
                    break;
                case NONE: drive(-175.0, -120);
                    break;
            }

            //drive to from blocks
            switch (skystonePosition)
            {
                case LEFT: drive(68, 0, false);
                    break;
                case CENTER: drive(68, 0.0, false);
                    break;
                case RIGHT: drive(68, 0.00, false);
                    break;
                case NONE: drive(68, 00.0, false);
                    break;
            }

            //grab block
            blockLifter.setPosition(BLOCK_LIFTER_DOWN);
            sleep(400);
            blockGrabber.setPosition(BLOCK_GRABBER_CLOSED);
            sleep(600); //changed from 250 milliseconds
            blockLifter.setPosition(BLOCK_LIFTER_UP);

            //drive away from blocks
            switch (skystonePosition)
            {
                case LEFT: drive(60, 0, false);
                    break;
                case CENTER: drive(60, 0.0, false);
                    break;
                case RIGHT: drive(60, 0.00, false);
                    break;
                case NONE: drive(60, 00.0, false);
                    break;
            }

            //TODO: Center and Right
            switch (skystonePosition)
            {
                case LEFT: drive(175, 120);
                    break;
                case CENTER: drive(195, 140);
                    break;
                case RIGHT: drive(215, 160);
                    break;
                case NONE: drive(170.0, 120);
                    break;
            }

            blockLifter.setPosition(BLOCK_LIFTER_LIFTED);
            blockGrabber.setPosition(BLOCK_GRABBER_OPEN);

            sleep(500);
        }
        else
        {
            /*
            drive(-190, -140);

            turn(90,false);
            turn(90, false);
            localization.resetPosition();

            drive(-20.0,0,false);

            intake(true);

            drive(0.,8,false);

            claw.setPosition(CLAW_CLOSED);

            drive(0., -8, false);

            drive (20.,0,false);

            drive(190, 140);

            while (lift.getCurrentPosition() < CLAW_HIGHEST_POSITION)
            {
                lift.setPower(0.5);
            }
            lift.setPower(0);
            clawArm.setPosition(CLAW_ARM_OUT);
            sleep(500);
            claw.setPosition(CLAW_OPEN);
            sleep(250);
            clawArm.setPosition(CLAW_ARM_IN);
            sleep(100);
            while (lift.getCurrentPosition() > CLAW_HIGHEST_POSITION)
            {
                lift.setPower(-.5);
            }
             */
        }

        blockLifter.setPosition(BLOCK_LIFTER_UP);

        driveUntilLine("blue", -0.5);
    }

    private void intake (boolean in)
    {
        if (in)
        {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        }
        else
        {
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
        }
    }

    private void drive (double x, double y, boolean foundation)
    {
        double  driveInput, strafeInput, turnInput,
                frontLeftPower, frontRightPower, backLeftPower, backRightPower;

        x*=10;
        y*=10;

        double xStartPosition = localization.getxPosition();
        double yStartPosition = localization.getyPosition();

        double xTargetPosition = xStartPosition + x;
        double yTargetPosition = yStartPosition + y;
        double targetAngle = 0;

        double dxTotal = Math.abs(x) > 60? xTargetPosition - xStartPosition : 100;
        double dyTotal = Math.abs(y) > 60? yTargetPosition - yStartPosition : -100;
        double dATotal = Math.PI/6;

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

            xError = notNaN((dx)/dxTotal);
            yError = notNaN((dy)/dyTotal);
            angleError = notNaN((dA)/ (dATotal));

            driveInput = y > 0 ? yError : -yError;//*cosine(Math.toRadians(angle)) + xError*sine(Math.toRadians(angle));
            turnInput  = 0.95*angleError;
            strafeInput = 0.95*(x > 0 ? xError : -xError);//*cosine(Math.toRadians(angle)) + yError*sine(Math.toRadians(angle));

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

            if (dx > -10 && dx < 10 && dy > -10 && dy < 10)
            {
                targetAcheived = true;
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
            }
        }
    }

    private void drive (float xTarget, double y, boolean foundation)
    {
        double  driveInput, strafeInput, turnInput,
                frontLeftPower, frontRightPower, backLeftPower, backRightPower;

        xTarget*=10;
        y*=10;

        double xStartPosition = localization.getxPosition();
        double yStartPosition = localization.getyPosition();

        double xTargetPosition = xTarget;
        double yTargetPosition = yStartPosition + y;
        double targetAngle = 0;

        double dxTotal = Math.abs(xTargetPosition-xStartPosition) > 60? xTargetPosition - xStartPosition : 100;
        double dyTotal = Math.abs(yTargetPosition-yStartPosition) > 60? yTargetPosition - yStartPosition : -100;
        double dATotal = Math.PI/6;

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

            xError = notNaN((dx)/dxTotal);
            yError = notNaN((dy)/dyTotal);
            angleError = notNaN((dA)/ (dATotal));

            driveInput = dyTotal > 0 ? yError : -yError;//*cosine(Math.toRadians(angle)) + xError*sine(Math.toRadians(angle));
            turnInput  = 0.95*angleError;
            strafeInput = 0.95*(dxTotal > 0 ? xError : -xError);//*cosine(Math.toRadians(angle)) + yError*sine(Math.toRadians(angle));

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

            if (dx > -10 && dx < 10 && dy > -10 && dy < 10)
            {
                targetAcheived = true;
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
            }
        }
    }

    private void drive (double distance, double distanceBeforeDecel)
    {
        double xStartPosition = localization.getxPosition();
        double yStartPosition = localization.getyPosition();
        distance *= 10;
        distanceBeforeDecel *= 10;

        double endPosition = localization.getyPosition() + distanceBeforeDecel;
        double turnCorrection, strafeCorrection;

        if (distance > 0)
        {
            while (localization.getyPosition() <= endPosition && opModeIsActive())
            {
                localization.update();
                turnCorrection = localization.getAngle()/(Math.PI/8);
                strafeCorrection = (xStartPosition - localization.getxPosition())/200;

                frontLeft.setPower(1 + turnCorrection + strafeCorrection);
                frontRight.setPower(1 - turnCorrection - strafeCorrection);
                backRight.setPower(1 + turnCorrection + strafeCorrection);
                backLeft.setPower(1 - turnCorrection - strafeCorrection);
            }
        }
        else
        {
            while (localization.getyPosition() >= endPosition && opModeIsActive())
            {
                localization.update();
                turnCorrection = localization.getAngle()/(Math.PI/8);
                strafeCorrection = (xStartPosition-localization.getxPosition())/200;

                frontLeft.setPower(-1 + turnCorrection + strafeCorrection);
                frontRight.setPower(-1 - turnCorrection - strafeCorrection);
                backRight.setPower(-1 + turnCorrection + strafeCorrection);
                backLeft.setPower(-1 - turnCorrection - strafeCorrection);
            }
        }

        double  driveInput, strafeInput, turnInput,
                frontLeftPower, frontRightPower, backLeftPower, backRightPower;

        yStartPosition += distanceBeforeDecel;

        double deltaDistance = (Math.abs(distance)-Math.abs(distanceBeforeDecel));

        double xTargetPosition = xStartPosition;
        double yTargetPosition = yStartPosition + (deltaDistance*(distance/Math.abs(distance)));
        double targetAngle = 0;

        double dxTotal = 200;
        double dyTotal = deltaDistance;
        double dATotal = Math.PI/8;

        double dx ,dy, dA;

        double xError, yError, angleError;

        boolean targetAcheived = false;

        double minPower = MIN_TURN_PWR;

        while (opModeIsActive() && !targetAcheived)
        {
            localization.update();

            dx = xTargetPosition - localization.getxPosition();
            dy = yTargetPosition - localization.getyPosition();
            dA = targetAngle - localization.getAngle();

            xError = notNaN((dx)/dxTotal);
            yError = notNaN((dy)/dyTotal);
            angleError = notNaN((dA)/ (dATotal));

            driveInput = yError;//*cosine(Math.toRadians(angle)) + xError*sine(Math.toRadians(angle));
            turnInput  = 0.95*angleError;
            strafeInput = 0.95*xError;//*cosine(Math.toRadians(angle)) + yError*sine(Math.toRadians(angle));

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

            if (dx > -10 && dx < 10 && dy > -10 && dy < 10)
            {
                targetAcheived = true;
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
            }
        }
    }

    private void driveWithoutAccuracy (double distanceCM, double power)
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

    private void turn (double angle, boolean foundation)
    {
        double  turnInput,
                frontLeftPower, frontRightPower, backLeftPower, backRightPower;

        angle = Math.toRadians(angle);

        double startAngle = localization.getAngle();

        double targetAngle = startAngle + angle;

        double dATotal = targetAngle - startAngle;

        double dA;

        double angleError;

        boolean targetAcheived = false;

        double minPower = foundation? 0.3 : MIN_TURN_PWR;

        while (opModeIsActive() && !targetAcheived)
        {
            localization.update();

            dA = targetAngle - localization.getAngle();

            angleError = notNaN((dA)/ (Math.abs(dATotal)));

            turnInput  = 1.2*angleError;

            frontLeftPower = moreThanMin(turnInput, minPower);
            frontRightPower = moreThanMin(-turnInput, minPower);
            backLeftPower = moreThanMin(turnInput, minPower);
            backRightPower = moreThanMin(-turnInput, minPower);

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
            backLeft.setPower(backLeftPower);

            telemetry.addData("turnInput", turnInput);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("angle", Math.toDegrees(localization.getAngle()));
            telemetry.addData("angleError", angleError);
            telemetry.addData("dA", dA);
            telemetry.update();

            if (Math.abs(angleError) < MIN_ANGLE_ERROR)
            {
                targetAcheived = true;
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
            }
        }
    }

    private void driveUntilLine (String lineColor, double power)
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

    private void strafe (double distanceCM, boolean right, double power)
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

    @Override
    public void runOpMode() throws InterruptedException
    {
        //start counting time
        runtime = new ElapsedTime();

        //locate drive motors from REV hub setup//
        frontLeft   = hardwareMap.get(DcMotor.class, "frontLeft");  //front left wheel
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight"); //front right wheel
        backLeft    = hardwareMap.get(DcMotor.class, "backLeft");   //back left wheel
        backRight   = hardwareMap.get(DcMotor.class, "backRight");  //back right wheel

        //locate other motors from REV hub setup//
        intakeLeft  = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get (DcMotor.class, "intakeRight");
        lift        = hardwareMap.get(DcMotor.class, "lift");
        rearEncoder = hardwareMap.get(DcMotor.class, "rearEncoder");

        //locate servos from REV hub setup//
        claw                = hardwareMap.servo.get("claw");
        clawArm             = hardwareMap.servo.get("clawArm");
        capstoneArm         = hardwareMap.servo.get("capstoneArm");
        capstoneHook        = hardwareMap.servo.get("capstoneHook");
        blockGrabber        = hardwareMap.servo.get("blockGrabber");
        blockLifter         = hardwareMap.servo.get("blockLifter");
        leftFoundGrabber    = hardwareMap.servo.get("foundGrabberLeft");
        rightFoundGrabber   = hardwareMap.servo.get("foundGrabberRight");

        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");

        rightEncoder = intakeRight;
        leftEncoder = intakeLeft;

        //set drive motor directions correctly//
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //set other motor directions correctly//
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);

        //reset and initialize lift motor//
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        //display camera monitor
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //initialize skystone detector pipeline
        skystoneDetector = new SkystoneDetectorPipeline();
        //locate camera from the USB hub
        webcam = OpenCvCameraFactory.getInstance().createWebcam
                (hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //open camera
        webcam.openCameraDevice();
        //set pipeline
        webcam.setPipeline(skystoneDetector);
        //start streaming image from webcam to robot controller phone
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        //start the position calculation system
        localization = new Odometry(rightEncoder, leftEncoder, rearEncoder);

        //set the servos to their starting positions
        initialize();

        telemetry.addData("Ready for parking Position", "");
        telemetry.addData("Far Park", "Up");
        telemetry.addData("Close Park", "Down");
        telemetry.update();

        int runMode = -1;

        while(runMode < 0 && !opModeIsActive())
        {
            if (gamepad1.dpad_up)
            {
                farPark = true;
                telemetry.addData("Park", "Far ");
                telemetry.addData("Select Using", "A");
                telemetry.update();
            }
            else if (gamepad1.dpad_down)
            {
                farPark = false;
                telemetry.addData("Park", "Close ");
                telemetry.addData("Select Using", "A");
                telemetry.update();
            }
            else if (gamepad1.a)
            {
                runMode = 1;
            }
        }

        //until select is let go
        while (gamepad1.a && !opModeIsActive()) {sleep(5);}

        telemetry.addData("Ready for FOUNDATION grabbing", "");
        telemetry.addData("Grab Foundation", "Up");
        telemetry.addData("Don't Grab Foundation", "Down");
        telemetry.update();

        int runMode2 = -1;

        while (runMode2 < 0 && !opModeIsActive())
        {
            if (gamepad1.dpad_up)
            {
                moveFoundation = true;
                telemetry.addData("Move Foundation", "Yes");
                telemetry.addData("Select Using", "B");
                telemetry.update();
            }
            else if (gamepad1.dpad_down)
            {
                moveFoundation = false;
                telemetry.addData("Move Foundation", "No");
                telemetry.addData("Select Using", "B");
                telemetry.update();
            }
            else if (gamepad1.b)
            {
                runMode2 = 1;
            }
        }

        //until select is let go
        while (gamepad1.b && !opModeIsActive()) {sleep(5);}

        telemetry.addData("Park", farPark ? "Far" : "Close");
        telemetry.addData("Move Foundation", moveFoundation ? "Yes" : "No");

        telemetry.addData("Ready for BLOCK grabbing", "");
        telemetry.addData("Grab Blocks", "Up");
        telemetry.addData("Don't Grab Blocks", "Down");
        telemetry.update();

        int runMode3 = -1;
        while (runMode3 < 0 && !opModeIsActive())
        {
            if (gamepad1.dpad_up)
            {
                getBlocks = true;
                telemetry.addData("Move Blocks", "Yes");
                telemetry.addData("Select Using", "A");
                telemetry.update();
            }
            else if (gamepad1.dpad_down)
            {
                getBlocks = false;
                telemetry.addData("Move Blocks", "No");
                telemetry.addData("Select Using", "A");
                telemetry.update();
            }
            else if (gamepad1.a)
            {
                runMode3 = 1;
            }
        }

        //until select is let go
        while (gamepad1.a && !opModeIsActive()) {sleep(5);}

        telemetry.addData("Park", farPark ? "Far" : "Close");
        telemetry.addData("Move Foundation", moveFoundation ? "Yes" : "No");
        telemetry.addData("Move Blocks", getBlocks ? "Yes" : "No");

        telemetry.addData("Ready for Position", "");
        telemetry.addData("Up", "Blue Build");
        telemetry.addData("Down", "Blue Blocks");
        telemetry.addData("Y", "Red Build");
        telemetry.addData("A", "Red Blocks");
        telemetry.update();

        runMode = -1;
        StartPosition pos = null;

        while (runMode < 0 && !opModeIsActive())
        {
            if (gamepad1.dpad_up)
            {
                pos = StartPosition.BLUE_BUILD;
                telemetry.addData("Position: ", "Blue Build");
                telemetry.addData("Select Using", "X");
                telemetry.update();
            }
            else if (gamepad1.dpad_down)
            {
                pos = StartPosition.BLUE_BLOCKS;
                telemetry.addData("Position: ", "Blue Blocks");
                telemetry.addData("Select Using", "X");
                telemetry.update();
            }
            else if (gamepad1.y)
            {
                pos = StartPosition.RED_BUILD;
                telemetry.addData("Position: ", "Red Build");
                telemetry.addData("Select Using", "X");
                telemetry.update();
            }
            else if (gamepad1.a)
            {
                pos = StartPosition.RED_BLOCKS;
                telemetry.addData("Position: ", "Red Blocks");
                telemetry.addData("Select Using", "X");
                telemetry.update();
            }
            else if (gamepad1.x)
            {
                runMode = 1;
            }
        }

        while (!isStarted()) {
            skystonePosition = skystoneDetector.getSkystonePosition();
            telemetry.addData("Park", farPark ? "Far" : "Close");
            telemetry.addData("Move Foundation", moveFoundation ? "Yes" : "No");
            telemetry.addData("Move Blocks", getBlocks ? "Yes" : "No");
            telemetry.addData("Position", pos.toString());
            telemetry.addData("Ready to Start!", "");
            telemetry.addData("Skystone Position", skystonePosition.toString());
            telemetry.update();
        }

        //reset the runtime
        runtime.reset();

        //move the foundation grabber out
        Thread setFoundationGrabber = new Thread (new FoundationGrabberSet(lift, leftFoundGrabber, rightFoundGrabber));
        setFoundationGrabber.start();

        executeAutonomous(pos);
    }
}
