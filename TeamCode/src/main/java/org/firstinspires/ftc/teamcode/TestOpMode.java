package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

@TeleOp(name="TestOpMode", group="Linear Opmode")
public class TestOpMode extends OpMode
{
    Robert rob = new Robert ();

    @Override
    public void init()
    {
        //show that rob is initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start()
    {
        rob.resetTime();
    }

    @Override
    public void loop()
    {
        rob.drive();
        rob.perform();
        addTelemetry(rob);
    }

    @Override
    public void stop()
    {

    }

    private void addTelemetry (Robert rob)
    {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + rob.getRuntimeAsString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rob.getFrontLeftPower(), rob.getBackLeftPower());
        telemetry.update();
    }
}
