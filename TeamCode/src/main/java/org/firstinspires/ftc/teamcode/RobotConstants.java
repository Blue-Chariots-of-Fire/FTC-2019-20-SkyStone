package org.firstinspires.ftc.teamcode;

public final class RobotConstants
{
    //Constants for motor/servo positions/////////
    static final double CLAW_ARM_IN             = 0.0;   //in
    static final double CLAW_ARM_OUT            = 1.0;   //in
    static final double CLAW_CLOSED             = 0.03;  //closed
    static final double CLAW_OPEN               = 1.0;   //open
    static final double CAPSTONE_ARM_OUT        = 0.9;   //out
    static final double CAPSTONE_ARM_IN         = 0.0;   //in
    static final double CAPSTONE_ARM_PARKED     = 0.4;   //parked
    static final double CAPSTONE_HOOK_HOOKED    = 1.0;   //hooked
    static final double CAPSTONE_HOOK_UN_HOOKED = 0.0;   //unhooked
    static final double BLOCK_LIFTER_UP         = 0.28;  //TODO: Check actual
    static final double BLOCK_LIFTER_DOWN       = 0.0;   //TODO: Check actual
    static final double BLOCK_GRABBER_OPEN      = 0.5;   //TODO: Check actual
    static final double BLOCK_GRABBER_CLOSED    = 0.0;   //TODO: Check actual
    static final double RIGHT_FOUND_UP          = 0.45;  //up
    static final double RIGHT_FOUND_STORED      = 0.0;   //stored
    static final double RIGHT_FOUND_DOWN        = 1.0;   //down
    static final double LEFT_FOUND_UP           = 0.0;   //up
    static final double LEFT_FOUND_DOWN         = 1.0;   //down
    static final double BLOCK_LIFTER_LIFTED     = 0.19;  //lifted

    //Threshold constants/////////////////////////
    static final double CLAW_HIGHEST_POSITION = 2200;    //lift must be this high to turn
    static final double LIFT_HIGHEST_POSITION = 2950;    //lift cannot exceed this height
    static final double LIFT_LOWEST_POSITION = 0;        //lift cannot exceed this height
    static final double SLOW_MODE_DIVISOR = 3;           //slow mode slows this much

    final static double MIN_ANGLE_ERROR = 0.01;
    final static double MIN_TURN_PWR = 0.17;

    //constants
    final static float ENCODER_TICKS_PER_REVOLUTION = 383.6f;
    final static float CIRCUMFERENCE_IN_CM = (float) Math.PI * 10.0f; //diameter is 100mm
    final static float WHEEL_GEAR_REDUCTION = 2f;
    final static float TICKS_PER_CM =  (ENCODER_TICKS_PER_REVOLUTION/CIRCUMFERENCE_IN_CM) * WHEEL_GEAR_REDUCTION; //Experimental Value: 33.69411764705882

    //Skystone position types
    enum SkystonePosition {LEFT, RIGHT, CENTER, NONE}

    static double sine (double angle)
    {
        double out;
        int count = 0;

        while (angle > Math.PI)
        {
            angle -= Math.PI;
            count++;
        }

        while (angle < 0.0)
        {
            angle += Math.PI;
            count++;
        }

        out = Math.sin(angle);

        return (count % 2 == 0 ? out : out*-1);
    }

    static double cosine (double angle)
    {
        double out;
        int count = 0;

        while (angle > Math.PI)
        {
            angle -= Math.PI;
            count++;
        }

        while (angle < 0.0)
        {
            angle += Math.PI;
            count++;
        }

        out = Math.cos(angle);

        return (count % 2 == 0 ? out : out*-1);
    }

    static double pythagorize (double a, double b)
    {
        return (Math.sqrt((a*a)+(b*b)));
    }

    static double derive (double fin, double init,  double time)
    {
        return ((fin-init)/time);
    }

    static double notNaN (double in)
    {
        return Double.isNaN(in) || Double.isInfinite(in) ? 0.0 : in;
    }

    static double moreThanMin (double in, double min)
    {
        if (Math.abs(in) > min)
        {
            return in;
        }
        else if (in > 0)
        {
            return min;
        }
        else
        {
            return -min;
        }
    }
}
