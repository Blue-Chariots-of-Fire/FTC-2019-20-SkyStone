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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    private DcMotor rearEncoder = null;
    private Servo claw = null;
    private Servo clawArm = null;
    private Servo foundGrabber = null;
    private Servo capstoneArm = null;
    private Servo capstoneHook = null;
    private BNO055IMU imu = null;
    private Orientation lastAngles = new Orientation();
    double globalAngle;
    private ModernRoboticsI2cColorSensor colorSensor = null;

    private Odometry localizer = null;


    // Constants for motor/servo positions
    private static final double clawArmIn = 0.0; //in
    private static final double clawArmOut = 1.0; //in
    private static final double foundationUp = 0.65; //up
    private static final double foundationDown = 0.0; //down
    private static final double clawClosed = 0.0; //closed
    private static final double clawOpen = 1.0; //open
    private static final double capstoneArmOut = 0.0; //out
    private static final double capstoneArmIn = 1.0; //in
    private static final double capstoneHookHooked = 0.0; //hooked
    private static final double capstoneHookUnHooked = 1.0; //unhooked

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
    private double clawPosition = clawOpen; //open
    private double clawArmPosition = clawArmIn; //in
    private double foundationPosition = foundationUp; //up
    private double capstoneArmPosition = capstoneArmOut;
    private double capstoneHookPosition = capstoneHookHooked;

    //variable for the controllers input
    private double turn;
    private double drive;
    private double strafe;
    private double liftAmount;

    //lift state machine booleans
    boolean moveUp = false;
    boolean moveOut = false;
    boolean moveIn = false;
    boolean clawCanTurn = false;

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
        rearEncoder = hardwareMap.get(DcMotor.class, "rearEncoder");
        foundGrabber = hardwareMap.servo.get("foundGrabber");
        claw = hardwareMap.servo.get("claw");
        clawArm = hardwareMap.servo.get("clawArm");
        capstoneArm = hardwareMap.servo.get("capstoneArm");
        capstoneHook = hardwareMap.servo.get("capstoneHook");
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");

        //imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

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
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        localizer = new Odometry(intakeRight, intakeLeft, rearEncoder);

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
            if (gamepad1.a)
            {
                resetAngle();
            }
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

        if (moveUp)
        {
            if (lift.getCurrentPosition() < 2200)
            {
                lift.setPower(1.0);
            }
            else
            {
                lift.setPower(0.0);
            }

            if (lift.getCurrentPosition() > 2200)
            {
                moveUp = false;
                clawCanTurn = true;
            }
        }

        if (clawCanTurn)
        {
            if (moveOut)
            {
                clawArmPosition = 1.0; //out
                moveOut = false;
                clawCanTurn = false;
            }

            if (moveIn)
            {
                clawArmPosition = 0.0; //in
                moveIn = false;
                clawCanTurn = false;
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
        liftAmount = -gamepad2.right_stick_y;
        liftPower = liftCutoff(liftAmount);
        if (!moveUp)
        {
            lift.setPower(liftPower);
        }
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
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    //adds telemetry info to the driver station
    private void telemetry ()
    {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("second angle", getAngle());
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("ARGB", colorSensor.argb());
        telemetry.update();
    }
}