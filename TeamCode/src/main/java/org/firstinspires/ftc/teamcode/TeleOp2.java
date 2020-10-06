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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



import static org.firstinspires.ftc.teamcode.RobotConstants.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp2", group="Linear Opmode")
public class TeleOp2 extends LinearOpMode
{
    //Declare OpMode members////////////////////
    private ElapsedTime runtime = new ElapsedTime();    //runtime counter
    private DcMotor frontLeft = null;                   //front left motor
    private DcMotor frontRight = null;                  //front right motor
    private DcMotor backLeft = null;                    //back left motor
    private DcMotor backRight = null;                   //back right motor

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

    public void runOpMode()
    {
        //locate drive motors from REV hub setup//
        frontLeft   = hardwareMap.get(DcMotor.class, "frontLeft");  //front left wheel
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight"); //front right wheel
        backLeft    = hardwareMap.get(DcMotor.class, "backLeft");   //back left wheel
        backRight   = hardwareMap.get(DcMotor.class, "backRight");  //back right wheel

        //set drive motor directions correctly//
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        //display initialized once ready//
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //wait for the game to start (driver presses PLAY)//
        waitForStart();
        runtime.reset();

        //run until the end of the opMode is not active (STOP or time)
        while (opModeIsActive())
        {
            //run drivetrain code//
            checkSlowMode();        //check if slow mode is active or not
            checkReverseDrive();    //check if reverse drive is active or not
            driveTrain();           //drive the drivetrain

            telemetry();
        }
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
     * adds telemetry info to the driver station
     */
    private void telemetry ()
    {
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Power", gamepad1.left_stick_y);

        telemetry.update();
    }
}