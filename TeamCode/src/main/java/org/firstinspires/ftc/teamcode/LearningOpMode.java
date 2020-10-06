package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpLearning", group="Linear Opmode")
public class LearningOpMode extends OpMode
{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    double turnInput, driveInput, strafeInput;

    public void init()
    {
        frontLeft   = hardwareMap.get(DcMotor.class, "frontLeft");  //front left wheel
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight"); //front right wheel
        backLeft    = hardwareMap.get(DcMotor.class, "backLeft");   //back left wheel
        backRight   = hardwareMap.get(DcMotor.class, "backRight");  //back right wheel
    }

    public void loop()
    {
        turnInput = gamepad1.right_stick_x;
        driveInput = gamepad1.left_stick_y;
        strafeInput = gamepad1.left_stick_x;

        frontLeft.setPower(driveInput + turnInput + strafeInput);
        frontRight.setPower(driveInput - turnInput - strafeInput);
        backLeft.setPower(driveInput + turnInput - strafeInput);
        backRight.setPower(driveInput - turnInput + strafeInput);
    }

}
