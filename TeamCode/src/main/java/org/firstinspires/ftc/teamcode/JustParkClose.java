package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

@Autonomous
public class JustParkClose extends LinearOpMode
{
    private Servo capstoneArm;
    private Servo capstoneHook;

    @Override
    public void runOpMode() throws InterruptedException
    {
        capstoneArm = hardwareMap.servo.get("capstoneArm");
        capstoneHook = hardwareMap.servo.get("capstoneHook");

        capstoneArm.setPosition(CAPSTONE_ARM_IN);
        capstoneHook.setPosition(CAPSTONE_HOOK_HOOKED);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        capstoneArm.setPosition(CAPSTONE_ARM_PARKED);
        sleep(5000);
    }
}
