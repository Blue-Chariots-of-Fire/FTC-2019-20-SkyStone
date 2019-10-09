package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Robert extends OpMode
{
    //members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    // Setup a variable for each drive wheel
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    //variables for driving
    private double drive;
    private double turn;
    private double strafe;

    //constructor
    public Robert ()
    {

    }

    public void resetTime ()
    {
        runtime.reset();
    }

    public void drive ()
    {
        //sets drive wheel variables to their desired things
        frontLeftPower = Range.clip((drive-turn+strafe), -1.0, 1.0);
        frontRightPower = Range.clip((drive+turn-strafe), -1.0, 1.0);
        backLeftPower = Range.clip((drive-turn-strafe), -1.0, 1.0);
        backRightPower = Range.clip((drive+turn+strafe), -1.0, 1.0);
    }

    public void perform ()
    {
        // Send calculated power to wheels
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
    }

    //accessors
    public String getRuntimeAsString ()
    {
        return runtime.toString();
    }

    public double getBackLeftPower()
    {
        return backLeftPower;
    }

    public double getFrontLeftPower()
    {
        return frontLeftPower;
    }

    public double getFrontRightPower ()
    {
        return frontRightPower;
    }

    public double getBackRightPower()
    {
        return backRightPower;
    }

    //abstract
    public void init ()
    {
        // Initialize the hardware variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        //Reverse motors that are positioned backwards
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop ()
    {
        //variable for each mode of movement
        drive = -gamepad1.left_stick_y;
        turn  =  -gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;
    }
}