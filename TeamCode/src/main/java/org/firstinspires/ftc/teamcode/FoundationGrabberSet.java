package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

public class FoundationGrabberSet implements Runnable
{
    DcMotor lift;
    Servo leftFoundGrabber, rightFoundGrabber;

    public FoundationGrabberSet (DcMotor lift, Servo leftFoundGrabber, Servo rightFoundGrabber)
    {
        this.leftFoundGrabber = leftFoundGrabber;
        this.rightFoundGrabber = rightFoundGrabber;
        this.lift = lift;
    }

    public void run ()
    {
        while (lift.getCurrentPosition() < 525)
        {
            lift.setPower(0.25);
        }

        lift.setPower(0);

        leftFoundGrabber.setPosition(LEFT_FOUND_UP);
        rightFoundGrabber.setPosition(RIGHT_FOUND_UP);

        while (lift.getCurrentPosition() > 0)
        {
            lift.setPower(-0.25);
        }

        lift.setPower(0);
    }
}
