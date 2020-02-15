package org.firstinspires.ftc.teamcode;

public class LocalizerThread implements Runnable
{
    Odometry localization;
    boolean update;

    public LocalizerThread (Odometry localization)
    {
        this.localization = localization;
        update = true;
    }

    @Override
    public void run()
    {
        while (update)
        {
            localization.update();
        }
    }

    public void stop ()
    {
        update = false;
    }
}
