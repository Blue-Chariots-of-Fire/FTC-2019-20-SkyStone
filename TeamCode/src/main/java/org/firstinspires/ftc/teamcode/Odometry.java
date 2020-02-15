package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

class Odometry
{
    //Diagram of Robot
    /*
               intake
         ___________________
        |                   |
   left||                   || right
        |                   |
        |                   |
        |           rear    |
        |             __    |
        |___________________|
                lift
     */

    /* Description
       FRONT is intake side
       BACK is lift side

       right increases towards lift/back
             decreases towards intake/front
       left increases towards lift/back
            decreases towards intake/front
     */

    //Declare Class Members///////////////////////
    private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private DcMotor backEncoder;
    private ElapsedTime runtime;

    //Encoder Position Variables//////////////////
    private double rightPosition;
    private double leftPosition;
    private double backPosition;

    private double oldRightPosition;
    private double oldLeftPosition;
    private double oldBackPosition;

    //Position Variables//////////////////////////
    private double xPosition    = 0.0;
    private double yPosition    = 0.0;
    private double angle        = 0.0;

    private double oldxPosition = 0.0;
    private double oldyPosition = 0.0;
    private double oldAngle = 0.0;

    //position, velocity, acceleration, jerk, snap, crackle, pop
    double[] derivs = new double[7];
    double[] xDerivs = new double[7], oldxDerivs = new double[7];
    double[] yDerivs = new double[7], oldyDerivs = new double[7];

    //Other Variables/////////////////////////////
    private double cycleDistance;       //distance moved in current cycle
    private double rightCycleDistance;  //right wheel distance moved in current cycle
    private double leftCycleDistance;   //left wheel distance moved in current cycle
    private double backCycleDistance;   //back wheel distance moved in current cycle
    private double rearOffset;          //rear wheel distance moved due to turning
    private double deltaAngle;          //change in angle
    private double time;          //current time
    private double oldTime = 0.0;       //time from last cycle
    private double deltaTime;           //time since last cycle

    //Constants///////////////////////////////////
    private static final float MM_PER_CM              = 10f;    //mm/m
    private static final float REAR_OFFSET            = 225f; //mm
    private static final float ENCODER_DISTANCE       = 390f; //mm
    private static final float COUNTS_PER_REVOLUTION  = 8192f;    //ticks/rev
    private static final float WHEEL_DIAMETER         = 72f;      //mm
    private static final float WHEEL_CIRCUMFERENCE    = (float) (WHEEL_DIAMETER * Math.PI); //C=PI*D
    private static final float TICKS_PER_MM           = COUNTS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;
    private static final float MM_PER_TICKS           = 1/TICKS_PER_MM;

    Odometry (DcMotor rightEncoder, DcMotor leftEncoder, DcMotor backEncoder)
    {
        this.rightEncoder = rightEncoder;
        this.leftEncoder = leftEncoder;
        this.backEncoder = backEncoder;

        rightPosition = -rightEncoder.getCurrentPosition();
        leftPosition = -leftEncoder.getCurrentPosition();
        backPosition = backEncoder.getCurrentPosition();

        runtime = new ElapsedTime();
        time = runtime.time();
    }

    void update ()
    {
        oldRightPosition = rightPosition;
        oldLeftPosition = leftPosition;
        oldBackPosition = backPosition;
        oldAngle = angle;
        oldTime = time;

        rightPosition = -rightEncoder.getCurrentPosition();
        leftPosition = -leftEncoder.getCurrentPosition();
        backPosition = backEncoder.getCurrentPosition();
        time = runtime.time();

        calculatePosition();
        calculateDerivatives();
    }

    private void calculatePosition ()
    {
        rightCycleDistance = (rightPosition - oldRightPosition)*MM_PER_TICKS;
        leftCycleDistance = (leftPosition - oldLeftPosition)*MM_PER_TICKS;
        cycleDistance = (rightCycleDistance+leftCycleDistance)/2;

        angle += (leftCycleDistance - rightCycleDistance)/ENCODER_DISTANCE;
        deltaAngle = (angle - oldAngle);

        rearOffset = deltaAngle*(REAR_OFFSET*TICKS_PER_MM);
        backCycleDistance = (-(backPosition - oldBackPosition) - rearOffset)*MM_PER_TICKS;

        xPosition += cycleDistance*sine(angle) + backCycleDistance*cosine(angle);
        yPosition +=cycleDistance*cosine(angle) + backCycleDistance*sine(angle);
    }

    private void calculateDerivatives ()
    {
        deltaTime = (time - oldTime);

        oldxDerivs = xDerivs;
        oldyDerivs = yDerivs;

        xDerivs[0] = xPosition;
        yDerivs[0] = yPosition;
        oldxDerivs[0] = oldxPosition;
        oldyDerivs[0] = oldyPosition;
        derivs[0] = pythagorize(xDerivs[0], yDerivs[0]);

        for (int i = 1; i < xDerivs.length; i++)
        {
            xDerivs[i] = derive(xDerivs[i-1], oldxDerivs[i-1], deltaTime);
            yDerivs[i] = derive(yDerivs[i-1], oldyDerivs[i-1], deltaTime);
            derivs[i] = pythagorize(xDerivs[i], yDerivs[i]);
        }
    }

    double getRightPosition ()
    {
        return rightPosition;
    }

    double getLeftPosition ()
    {
        return leftPosition;
    }

    double getBackPosition ()
    {
        return backPosition;
    }

    double getxPosition ()
    {
        return xPosition;
    }

    double getyPosition ()
    {
        return yPosition;
    }

    double getAngle ()
    {
        return angle;
    }

    void resetAngle ()
    {
        angle = 0;
    }

    void resetAngle (double angle)
    {
        this.angle = angle;
    }

    double getxDeriv (int deriv)
    {
        return xDerivs[deriv];
    }

    double getyDeriv (int deriv)
    {
        return yDerivs[deriv];
    }

    double getDeriv (int deriv)
    {
        return derivs[deriv];
    }
}
