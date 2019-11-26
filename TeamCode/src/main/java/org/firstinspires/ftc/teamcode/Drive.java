package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Drive
{
    //Setup a variable for each drive wheel
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private Telemetry telemetry;

    final float ENCODER_TICKS_PER_REVOLUTION = 537.6f;
    final float CIRCUMFERENCE_IN_CM = (float) Math.PI * 0.1f; //diameter is 100mm
    final float TICKS_PER_CM = ENCODER_TICKS_PER_REVOLUTION/CIRCUMFERENCE_IN_CM;
    final float STRAFE_TICKS_PER_CM = 268.8f;
    final float TICKS_PER_DEGREE = 4f;

    /**
     * the constructor
     * @param frontRight
     * @param frontLeft
     * @param backRight
     * @param backLeft
     */
    public Drive (DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, Telemetry telemetry)
    {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.telemetry = telemetry;
    }

    public void drive (double distanceCM, double power)
    {
        double currentPosition = frontLeft.getCurrentPosition();
        double targetPosition = currentPosition + distanceCM*TICKS_PER_CM;
        if (distanceCM > 0)
        {
            while (currentPosition < targetPosition)
            {
                frontRight.setPower(power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(power);
            }
        }
        else
        {
            while (currentPosition > targetPosition)
            {
                frontRight.setPower(-power);
                frontLeft.setPower(-power);
                backRight.setPower(-power);
                backLeft.setPower(-power);
            }
        }

        ctrlAltDel();
    }

    public void strafe (double distanceCM, boolean right, double power)
    {
        double currentPosition = frontLeft.getCurrentPosition();
        double targetPosition = currentPosition + distanceCM*TICKS_PER_CM;

        if (right)
        {
            while (currentPosition < targetPosition)
            {
                frontRight.setPower(power);
                frontLeft.setPower(-power);
                backRight.setPower(-power);
                backLeft.setPower(power);
            }
        }
        else
        {
            while (currentPosition > targetPosition)
            {
                frontRight.setPower(-power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(-power);
            }
        }

        ctrlAltDel();
    }

    public void turn (float angle, boolean CCW, double powah, BNO055IMU imu)
    {
        //#define powah power;
        double currentAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        double targetAngle;

        if (CCW)
        {
            targetAngle = currentAngle - angle;

            while (currentAngle > targetAngle)
            {
                frontRight.setPower(powah);
                frontLeft.setPower(-powah);
                backRight.setPower(powah);
                backLeft.setPower(-powah);
                telemetry.addData("fistAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("secondAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
                telemetry.addData("thirdAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
            }
        }
        else
        {
            targetAngle = currentAngle + angle;

            while (currentAngle < targetAngle)
            {
                frontRight.setPower(-powah);
                frontLeft.setPower(powah);
                backRight.setPower(-powah);
                backLeft.setPower(powah);
                telemetry.addData("fistAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("secondAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
                telemetry.addData("thirdAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
            }
        }

        ctrlAltDel();
    }

    public void ctrlAltDel ()
    {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
}
