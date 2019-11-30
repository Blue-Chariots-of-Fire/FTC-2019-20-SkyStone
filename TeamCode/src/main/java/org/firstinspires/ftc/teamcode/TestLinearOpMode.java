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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="TestTeleOpMode", group="Linear Opmode")
public class TestLinearOpMode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor intakeLeft = null;
    private DcMotor intakeRight = null;
    private DcMotor lift = null;
    private Servo claw = null;
    private Servo clawArm = null;
    private Servo foundGrabber = null;
    private Servo capstoneArm = null;
    private Servo capstoneHook = null;
    private BNO055IMU imu = null;

    //booleans
    private boolean slowMode = false;
    private boolean intake = false;
    private boolean intakeReverse = false;
    private boolean reverseDrive = false;

    // Setup a variable for each drive wheel
    private double frontRightPower = 0.0;
    private double frontLeftPower = 0.0;
    private double backLeftPower = 0.0;
    private double backRightPower = 0.0;
    private double intakePower = 0.0;
    private double liftPower = 0.0;
    private double clawPosition = 1.0;
    private double clawArmPosition = 0.0;
    private double foundationPosition = 0.65;
    private double capstoneArmPosition = 0.0;
    private double capstoneHookPosition = 0.0;

    //variable for the controllers
    private double turn;
    private double drive;
    private double strafe;
    private double liftAmount;

    //imu angles
    private double horizontalAngle;

    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get (DcMotor.class, "intakeRight");
        lift = hardwareMap.get(DcMotor.class, "lift");
        foundGrabber = hardwareMap.servo.get("foundGrabber");
        claw = hardwareMap.servo.get("claw");
        clawArm = hardwareMap.servo.get("clawArm");
        capstoneArm = hardwareMap.servo.get("capstoneArm");
        capstoneHook = hardwareMap.servo.get("capstoneHook");

        //imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //The REV Expansion hub is mounted vertically, so we have to flip the y and z axes.
        byte AXIS_MAP_CONFIG_BYTE = 0x18;
        byte AXIS_MAP_SIGN_BYTE = 0x1;

        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        try {
            Thread.sleep(100);
        }catch(InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        try {
            Thread.sleep(100);
        }catch(InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            checkIntake();
            checkSlowMode();
            driveTrain();
            lift();
            intake();
            claw();
            clawArm();
            checkReverseDrive();
            foundation();
            capstoneThingy();
            telemetry();
            /*
            telemetry.addData("fistAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("secondAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
            telemetry.addData("thirdAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
            telemetry.addLine("Lift Position: "+lift.getCurrentPosition());
            telemetry.addData("Claw Position: ", claw.getPosition());
            telemetry.addData("Claw Arm Position: ", clawArm.getPosition());
            telemetry.addData("Foundation Position: ", foundGrabber.getPosition());
             */
            telemetry.addData("secondAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
            telemetry.addData("frontLeftPosition: ", frontLeft.getCurrentPosition());
            telemetry.addData("frontRightPosition: ", frontRight.getCurrentPosition());
            telemetry.addData("backLeftPosition: ", backLeft.getCurrentPosition());
            telemetry.addData("backRightPosition: ", backRight.getCurrentPosition());
            telemetry.update();
        }
    }

    /**
     * controls the claw
     */
    private void claw ()
    {
        if (gamepad2.a)
        {
            clawPosition = 1.0; //let go
        }
        else if (gamepad2.b)
        {
            clawPosition = 0.0; //grab
        }

        claw.setPosition(clawPosition);
    }

    /**
     * controls the claw arm
     */
    private void clawArm ()
    {
        boolean moveOutState = false;

        if(gamepad2.x)
        {
            if (lift.getCurrentPosition() > 2300)
            {
                clawArmPosition = 1.0; //out
            }
            else
            {
                moveOutState = true;
            }
        }
        else if (gamepad2.y)
        {
            if (lift.getCurrentPosition() > 2300)
            {
                clawArmPosition = 0.0; //in
            }
        }

        if (moveOutState)
        {
            //go up until 2300
            if (lift.getCurrentPosition() < 2300)
            {
                lift.setPower (0.5);
            }

            if (lift.getCurrentPosition() > 2300) //it is above 2300
            {
                lift.setPower (0.0); //stop lift
                clawArmPosition = 1.0; // out
                moveOutState = false;
            }

        }

        clawArm.setPosition(clawArmPosition);
    }

    /**
     * controls the foundation grabber
     */
    private void foundation ()
    {
        if (gamepad2.dpad_up)
        {
            foundationPosition = 0.0;
        }
        else if (gamepad2.dpad_down)
        {
            foundationPosition = 0.65;
        }

        foundGrabber.setPosition(foundationPosition);
    }

    //checks if the robot is on slow mode and changes the mode
    private void checkSlowMode()
    {
        if (gamepad1.a)
        {
            slowMode = true;
        }

        if (gamepad1.b)
        {
            slowMode = false;
        }
    }

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

    private void capstoneThingy ()
    {
        if (gamepad2.dpad_left)
        {
            capstoneArmPosition = 0.0;
        }
        else
        {
            capstoneArmPosition = 1.0;
        }

        if (gamepad2.left_bumper && gamepad2.right_bumper)
        {
            capstoneHookPosition = 0.0;
        }
        else
        {
            capstoneHookPosition = 1.0;
        }

        capstoneArm.setPosition(capstoneArmPosition);
        capstoneHook.setPosition(capstoneHookPosition);
    }

    /**
     * checks if the intake should be on and in in what direction
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
     * cuts off the control for the lift if its position is out of bounds
     * @param in is the power from the controller
     * @return is the power to the motor
     */
    private double liftCutoff(double in)
    {
        if (lift.getCurrentPosition() > 2950)
        {
            return -0.10;
        }
        else if (lift.getCurrentPosition() < 0)
        {
            return 0.20;
        }
        else
        {
            return in;
        }
    }

    private void driveTrain ()
    {
        drive = -gamepad1.left_stick_y;
        turn  =  -gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        if (reverseDrive)
        {
            drive *= -1;
            strafe *= -1;
        }

        if (slowMode)
        {
            frontLeftPower = Range.clip((drive-turn+strafe), -1.0, 1.0);
            frontRightPower = Range.clip((drive+turn-strafe), -1.0, 1.0);
            backLeftPower = Range.clip((drive-turn-strafe), -1.0, 1.0);
            backRightPower = Range.clip((drive+turn+strafe), -1.0, 1.0);
        }
        if (!slowMode)
        {
            frontLeftPower = (drive-turn+strafe)/3;
            frontRightPower = (drive+turn-strafe)/3;
            backLeftPower = (drive-turn-strafe)/3;
            backRightPower = (drive+turn+strafe)/3;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
    }

    private void lift ()
    {
        if (gamepad2.left_stick_y == 0.0)
        {
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else
        {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        liftAmount = -gamepad2.right_stick_y;
        liftPower = liftCutoff(liftAmount);
        lift.setPower(liftPower);
    }

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

    //adds telemetry info to the driver station
    private void telemetry ()
    {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower);
        telemetry.update();
    }
}