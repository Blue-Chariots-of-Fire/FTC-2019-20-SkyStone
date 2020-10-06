/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.RobotConstants.BLOCK_GRABBER_CLOSED;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLOCK_GRABBER_OPEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLOCK_LIFTER_DOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BLOCK_LIFTER_UP;
import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_ARM_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_ARM_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_HOOK_HOOKED;
import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_HOOK_UN_HOOKED;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_ARM_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_ARM_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_HIGHEST_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_FOUND_DOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFT_FOUND_UP;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_HIGHEST_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_LOWEST_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_FOUND_DOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHT_FOUND_UP;
import static org.firstinspires.ftc.teamcode.RobotConstants.SLOW_MODE_DIVISOR;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpRecorder", group="Linear Opmode")
@Disabled
public class TeleOpRecorder extends OpMode
{
    //Declare OpMode members////////////////////
    private ElapsedTime runtime = new ElapsedTime();    //runtime counter
    private DcMotor frontLeft = null;                   //front left motor
    private DcMotor frontRight = null;                  //front right motor
    private DcMotor backLeft = null;                    //back left motor
    private DcMotor backRight = null;                   //back right motor
    private DcMotor intakeLeft = null;                  //left intake wheel motor
    private DcMotor intakeRight = null;                 //right intake wheel motor
    private DcMotor lift = null;                        //lift motor
    private DcMotor rearEncoder = null;                 //rear encoder
    private Servo claw = null;                          //claw servo
    private Servo clawArm = null;                       //claw arm servo
    private Servo capstoneArm = null;                   //capstone arm servo
    private Servo capstoneHook = null;                  //capstone hook servo
    private Servo blockLifter = null;                   //block lifter servo
    private Servo blockGrabber = null;                  //block grabber servo
    private Servo leftFoundGrabber = null;              //left foundation grabber
    private Servo rightFoundGrabber = null;             //right foundation grabber
    private BNO055IMU imu = null;                       //REV Hub internal motion unit

    private ArrayList <ControllerState> controllerStates;

    //localizer //////////////////////////////////
    private Odometry odometry = null;                   //odometry system

    //Mode booleans///////////////////////////////
    private boolean slowMode        = false;    //slow mode starts false
    private boolean reverseDrive    = false;    //reverse drive mode starts false
    private boolean intake          = false;    //intake starts off
    private boolean intakeReverse   = false;    //reverse intake starts off

    //Setup a variable for each motor power///////
    private double frontRightPower      = 0.0;  //front right motor power starts 0
    private double frontLeftPower       = 0.0;  //front left motor power starts 0
    private double backLeftPower        = 0.0;  //back left motor power start 0
    private double backRightPower       = 0.0;  //back right motor power starts 0
    private double intakePower          = 0.0;  //intake motors power starts 0
    private double liftPower            = 0.0;  //lift motor power starts 0

    //Setup a variable for each servo position///////
    private double clawPosition         = CLAW_OPEN;                //claw start open
    private double capstoneArmPosition  = CAPSTONE_ARM_OUT;         //capstone arm starts out
    private double capstoneHookPosition = CAPSTONE_HOOK_HOOKED;     //capstone hook starts hooked
    private double clawArmPosition      = CLAW_ARM_IN;              //clawArm starts in
    private double leftFoundPosition    = LEFT_FOUND_UP;            //left foundation starts up
    private double rightFoundPosition   = RIGHT_FOUND_UP;           //right foundation starts down
    private double blockLifterPosition  = BLOCK_LIFTER_UP;          //block lifter starts up
    private double blockGrabberPosition = BLOCK_GRABBER_CLOSED;      //block grabber starts closed

    //variable for the controllers input//////////
    private double turnInput    = 0.0;  //turn amount starts 0
    private double driveInput   = 0.0;  //drive amount starts 0
    private double strafeInput  = 0.0;  //strafe amount starts 0
    private double liftInput    = 0.0;  //lift amount stats 0

    //lift state machine booleans/////////////////
    boolean moveUp      = false;    //true while lift moves up during claw arm movement
    boolean moveOut     = false;    //true while claw arm moves out
    boolean moveIn      = false;    //true while claw arm moves in
    boolean clawCanTurn = false;    //true while lift is above can turn threshold

    public void init ()
    {
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

        //display initialized once ready//
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        odometry = new Odometry(intakeRight, intakeLeft, rearEncoder);

        controllerStates = new ArrayList <ControllerState> ();
    }

    public void loop ()
    {
        controllerStates.add(new ControllerState(gamepad1));
        controllerStates.add(new ControllerState(gamepad2));

        //update localizer//
        odometry.update();

        //run drivetrain code//
        checkSlowMode();        //check if slow mode is active or not
        checkReverseDrive();    //check if reverse drive is active or not
        driveTrain();           //drive the drivetrain

        //run intake code//
        checkIntake();  //check what the intake should be doing
        intake();       //do what the intake should be doing

        sideBlock();

        //run lift code//
        lift();     //run lift motor operations
        claw();     //run claw servo operations
        clawArm();  //run clawArm servo operations

        //run other parts code//
        foundation();       //run foundation grabber operations
        capstoneThingy();   //run capstone arm and hook operations

        //update driver station with stats//
        telemetry();    //display troubleshooting numbers
    }

    @Override
    public void stop ()
    {

    }

    private void sideBlock()
    {
        if (gamepad1.left_bumper)
        {
            blockLifterPosition = BLOCK_LIFTER_DOWN;
        } else if (gamepad1.right_bumper)
        {
            blockLifterPosition = BLOCK_LIFTER_UP;
        }

        if (gamepad1.right_stick_button)
        {
            blockGrabberPosition = BLOCK_GRABBER_OPEN;
        }
        else if (gamepad1.left_stick_button)
        {
            blockGrabberPosition = BLOCK_GRABBER_CLOSED;
        }
        blockLifter.setPosition(blockLifterPosition);
        blockGrabber.setPosition(blockGrabberPosition);
    }

    /**
     * second operator controls the claw
     * GP2-A opens the claw
     * GP2-B closes the claw
     */
    private void claw ()
    {
        if (gamepad2.a)
        {
            clawPosition = CLAW_OPEN; //let go
        }
        else if (gamepad2.b)
        {
            clawPosition = CLAW_CLOSED; //grab
        }
        claw.setPosition(clawPosition);
    }

    /**
     * second operator controls the claw arm
     * PD2-X moves claw arm out
     * GP2-Y moves claw arm in
     */
    private void clawArm ()
    {
        //begin state machine if a button is pressed
        if(gamepad2.x)
        {
            moveUp = true;
            moveOut = true;
        }
        else if (gamepad2.y)
        {
            moveUp = true;
            moveIn = true;
        }

        //if a button has been pressed, move lift up
        if (moveUp)
        {
            //move up as long as under claw CanTurnThreshold
            if (lift.getCurrentPosition() < CLAW_HIGHEST_POSITION)
            {
                lift.setPower(1.0);
            }
            else
            {
                lift.setPower(0.0);
            }

            //if lift has reach claw CanTurnThreshold, move on to next state
            if (lift.getCurrentPosition() > CLAW_HIGHEST_POSITION)
            {
                moveUp = false;
                clawCanTurn = true;
            }
        }

        //if lift is up, turn the claw the intended direction
        if (clawCanTurn)
        {
            if (moveOut)
            {
                clawArmPosition = CLAW_ARM_OUT;
                moveOut = false;
                clawCanTurn = false;
            }

            if (moveIn)
            {
                clawArmPosition = CLAW_ARM_IN;
                moveIn = false;
                clawCanTurn = false;
            }
        }
        clawArm.setPosition(clawArmPosition);
    }

    /**
     * second driver controls the foundation grabber
     * GP2-Up puts the foundation up
     * GP2-Down puts the foundation down
     */
    private void foundation ()
    {
        if (gamepad2.dpad_up)
        {
            rightFoundPosition = RIGHT_FOUND_UP;
            leftFoundPosition = LEFT_FOUND_UP;
        }
        else if (gamepad2.dpad_down)
        {
            rightFoundPosition = RIGHT_FOUND_DOWN;
            leftFoundPosition = LEFT_FOUND_DOWN;
        }

        rightFoundGrabber.setPosition(rightFoundPosition);
        leftFoundGrabber.setPosition(leftFoundPosition);
    }

    /**
     * checks if the robot is on slow mode and changes the mode
     * primary driver controls the slow mode mode
     * GP1-A sets fast mode
     * GP1-B sets slow mode
     */
    private void checkSlowMode()
    {
        if (gamepad1.a)
        {
            slowMode = false;
        }

        if (gamepad1.b)
        {
            slowMode = true;
        }
    }

    /**
     * checks if the lift should be the front of the robot
     * primary driver controls reverse drive mode
     * GP1-X sets intake as front
     * GP1-Y sets lift as front
     */
    private void checkReverseDrive()
    {
        if(gamepad1.x)
        {
            reverseDrive = false;
        }
        else if (gamepad1.y)
        {
            reverseDrive = true;
        }
    }

    /**
     * secondary controller controls the capstone assembly
     * GP2-(BOTH BUMPERS) drops capstone, otherwise remains held
     * GP2-Left moves arm in while pressed, otherwise arm stays out
     */
    private void capstoneThingy ()
    {
        if (gamepad2.dpad_left)
        {
            capstoneArmPosition = CAPSTONE_ARM_IN;
        }
        else
        {
            capstoneArmPosition = CAPSTONE_ARM_OUT;
        }

        if (gamepad2.left_bumper && gamepad2.right_bumper)
        {
            capstoneHookPosition = CAPSTONE_HOOK_UN_HOOKED;
        }
        else
        {
            capstoneHookPosition = CAPSTONE_HOOK_HOOKED;
        }
        capstoneArm.setPosition(capstoneArmPosition);
        capstoneHook.setPosition(capstoneHookPosition);
    }

    /**
     * primary diver controls if the intake should be on and in in what direction
     * GP1-Up pushes blocks OUT
     * GP1-Down pushes blocks IN
     */
    private void checkIntake ()
    {
        if (gamepad1.dpad_down)
        {
            intakeReverse = false;
            intake = true;
        }

        if (gamepad1.dpad_up)
        {
            intake = false;
            intakeReverse = true;
        }

        if (gamepad1.dpad_right)
        {
            intake = false;
            intakeReverse = false;
        }
    }

    /**
     * secondary operator controls lift
     * GP2-LS-Y moves lift
     */
    private void lift ()
    {
        liftInput = -gamepad2.right_stick_y;
        liftPower = liftCutoff(liftInput);

        //as to not interfere with claw state machine
        if (!moveUp)
        {
            lift.setPower(liftPower);
        }
    }

    /**
     * cuts off the control for the lift if its position is out of bounds
     * @param in is the power from the controller
     * @return is the power to the motor
     */
    private double liftCutoff(double in)
    {
        if (lift.getCurrentPosition() > LIFT_HIGHEST_POSITION)
        {
            return in;
        }
        else if (lift.getCurrentPosition() < LIFT_LOWEST_POSITION)
        {
            return 0.20;
        }
        else
        {
            return in;
        }
    }

    /**
     * primary operator controls the drivetrain
     * GP1-RS-Y drives robot
     * GP1-RS-X strafes robot
     * GP1-LS-X turns robot
     */
    private void driveTrain ()
    {
        //get inputs from controller
        driveInput = -gamepad1.left_stick_y;
        turnInput  =  -gamepad1.right_stick_x;
        strafeInput = gamepad1.left_stick_x + gamepad1.right_trigger/3 - gamepad1.left_trigger/3;

        //opposite drive and strafe if lift is lift is front
        if (reverseDrive)
        {
            driveInput *= -1;
            strafeInput *= -1;
        }

        //full speed while not slow mode
        if (!slowMode) {
            frontLeftPower = Range.clip((driveInput - turnInput + strafeInput), -1.0, 1.0);
            frontRightPower = Range.clip((driveInput + turnInput - strafeInput), -1.0, 1.0);
            backLeftPower = Range.clip((driveInput - turnInput - strafeInput), -1.0, 1.0);
            backRightPower = Range.clip((driveInput + turnInput + strafeInput), -1.0, 1.0);
        } else if (slowMode) //one-third speed if slow mode
        {
            frontLeftPower = (driveInput-turnInput+strafeInput)/ SLOW_MODE_DIVISOR;
            frontRightPower = (driveInput+turnInput-strafeInput)/ SLOW_MODE_DIVISOR;
            backLeftPower = (driveInput-turnInput-strafeInput)/ SLOW_MODE_DIVISOR;
            backRightPower = (driveInput+turnInput+strafeInput)/ SLOW_MODE_DIVISOR;
        }

        //set motors to their corresponding power
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
    }

    /**
     * controls the intake wheels based on the booleans set by checkIntake()
     */
    private void intake ()
    {
        if (intake)
        {
            intakePower = 1.0;
        }
        else if (intakeReverse)
        {
            intakePower = -1.0;
        }
        else
        {
            intakePower = 0.0;
        }

        // Send calculated power to motors
        intakeLeft.setPower(intakePower);
        intakeRight.setPower(intakePower);
    }

    /**
     * adds telemetry info to the driver station
     */
    private void telemetry ()
    {
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("FrontLeftPosition", frontLeft.getCurrentPosition());

        telemetry.addData("Right Encoder Position", odometry.getRightPosition());
        telemetry.addData("Left Encoder Position", odometry.getLeftPosition());
        telemetry.addData("Read Encoder Position", odometry.getBackPosition());

        telemetry.addData("xPosition", odometry.getxPosition());
        telemetry.addData("yPosition", odometry.getyPosition());
        telemetry.addData("Angle", Math.toDegrees(odometry.getAngle()));

        telemetry.addData("Power", gamepad1.left_stick_y);

        telemetry.update();
    }
}