package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

@Autonomous(name = "Skystone Autonomous")
@Disabled
public class SkystoneAutonomous extends LinearOpMode
{
    private OpenCvCamera webcam;
    private SkystoneDetectorPipeline detector;
    private SkystonePosition position;

    @Override
    public void runOpMode ()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        detector = new SkystoneDetectorPipeline();

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(detector);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (!isStarted()) {
            position = detector.getSkystonePosition();
            telemetry.addData("position", position);
            telemetry.update();
        }

    }
}

