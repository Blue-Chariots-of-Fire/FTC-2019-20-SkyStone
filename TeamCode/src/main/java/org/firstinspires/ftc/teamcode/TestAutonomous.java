/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="TestAutoOpMode", group="Linear OpMode")
@Disabled
public class TestAutonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor intakeLeft = null;
    private DcMotor intakeRight = null;
    private DcMotor lift = null;
    private Servo claw = null;
    private Servo clawArm = null;
    private Servo foundGrabber = null;
    private Servo capstoneArm = null;
    private Servo capstoneHook = null;

    private BNO055IMU imu = null;

    //object detection and localization stuff
    private static final String VUFORIA_KEY =
            "AWzLk7z/////AAABmddNYTiQD09gpN3oA3v0doxk89BClNkgrwp6vye9ZHmvHIMpmhAWWSZfuIoq6RtEk+lN"
                    + "DQFXTTR98qWs/Q3eKEgjeTZW6hnjMVYMFYZYOqkCqkMSVsn782g6Fc1504xmO42MLdaGmzi9EQ"
                    + "edbZmGYqWZFBt2A84IIT6S6SZTUFDOe0G7Wl/T2zn5CpcY2Cf5GloRtwSbIr4hhUj4fHU4DGf7"
                    + "cc9XbfbbVqjWSP4aM/FzQuJqnTvHsp/yiEvwV0fjfN6NCptgCyfGmHdh4L3NOtXklHFXo0SGV2"
                    + "+/t1td//GBks5RPXDzwE6ODGSndTdkW1dr5U6ZE25niSN3Mz4fJRx4uRtNRCd41ZG9U72qDAWr";
    // Class Members
    private VuforiaLocalizer vuforia;
    private OpenGLMatrix lastLocation = null;
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Constants for motor/servo positions
    private static final double clawArmIn = 0.0; //in
    private static final double clawArmOut = 1.0; //in
    private static final double foundationUp = 0.65; //up
    private static final double foundationDown = 0.0; //down
    private static final double clawClosed = 0.0; //closed
    private static final double clawOpen = 1.0; //open

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //constants
    final float ENCODER_TICKS_PER_REVOLUTION = 537.6f;
    final float CIRCUMFERENCE_IN_CM = (float) Math.PI * 10.0f; //diameter is 100mm
    final float TICKS_PER_CM =  33.69411764705882f; //ENCODER_TICKS_PER_REVOLUTION/CIRCUMFERENCE_IN_CM; //Experimental Value: 33.69411764705882
    final float STRAFE_TICKS_PER_CM = 35.77f;
    final float TICKS_PER_DEGREE = 4f;

    private enum StartPosition {RED_BLOCKS, BLUE_BLOCKS, RED_BUILD, BLUE_BUILD};

    public void runOpMode()
    {
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // Initialize the hardware variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get (DcMotor.class, "intakeRight");
        lift = hardwareMap.get(DcMotor.class, "lift");
        foundGrabber = hardwareMap.servo.get("foundGrabber");
        claw = hardwareMap.servo.get("claw");
        clawArm = hardwareMap.servo.get("clawArm");
        capstoneArm = hardwareMap.servo.get("capstoneArm");
        capstoneHook = hardwareMap.servo.get("capstoneHook");

        // reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // run using encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        //imu
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        params.calibrationDataFile = "BNO055IMUCalibration.json";
        params.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
        //The REV Expansion hub is mounted vertically, so we have to flip the y and z axes.
        byte AXIS_MAP_CONFIG_BYTE = 0x18;
        byte AXIS_MAP_SIGN_BYTE = 0x1;

        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        try {
            Thread.sleep(100);
        }catch(InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        try {
            Thread.sleep(100);
        }catch(InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }

        initializeRobot();

        boolean flag = true;
        StartPosition pos = null;

        while (flag)
        {

            if (gamepad1.dpad_up)
            {
                pos = StartPosition.BLUE_BUILD;
                telemetry.addData("Position: ", "Blue Build");
                telemetry.update();
            }
            else if (gamepad1.dpad_down)
            {
                pos = StartPosition.BLUE_BLOCKS;
                telemetry.addData("Position: ", "Blue Blocks");
                telemetry.update();
            }
            else if (gamepad1.y)
            {
                pos = StartPosition.RED_BUILD;
                telemetry.addData("Position: ", "Red Build");
                telemetry.update();
            }
            else if (gamepad1.a)
            {
                pos = StartPosition.RED_BLOCKS;
                telemetry.addData("Position: ", "Red Blocks");
            }

            if (gamepad1.back)
            {
                flag = false;
                telemetry.addLine("Start Position Set!");
                telemetry.update();
            }

            if (opModeIsActive())
            {
                flag = false;
                telemetry.addLine("Start Position NOT Set!");
                telemetry.update();
            }
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (pos == null)
        {

        }

        //executeAutonomous(pos);

        targetsSkyStone.activate();

        //go to skystone
        boolean tempFlagSystone = true;
        double tempSkystoneXPos = 0.0;
        double tempSkystoneYPos = 0.0;
        while (tempFlagSystone && !isStopRequested())
        {
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible)
            {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos ", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0), translation.get(1), translation.get(2));

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                tempSkystoneXPos = translation.get(0);
                tempSkystoneXPos *= -1.0;
                tempSkystoneXPos = 0.6798*tempSkystoneXPos + 4.1336;
                tempSkystoneXPos *= 0.1;

                tempSkystoneYPos = translation.get(1);
                tempSkystoneYPos = 0.8881*tempSkystoneYPos + 130.2;
                tempSkystoneYPos *= 0.1;

                tempFlagSystone = false;
            }
            else
            {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }


        telemetry.addData("Distance X To Go: ", tempSkystoneXPos);
        telemetry.addData("Distance Y To Go: ", tempSkystoneYPos);
        telemetry.update();


        if (tempSkystoneYPos > 0)
        {
            strafe(tempSkystoneYPos, true, 0.15);
        }
        else
        {
            strafe(tempSkystoneYPos*-1.0, false, 0.15);
        }
        drive(tempSkystoneXPos - 9, 0.15);
        moveLiftOut();
        claw.setPosition(clawClosed);
        sleep(1000);
        liftLift(250, 0.25);
        drive(-30, 0.5);
        strafe(30, true, 0.5);

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }

    private void executeAutonomous (StartPosition pos)
    {
        switch (pos)
        {
            case RED_BLOCKS: doRedBlock();
                break;
            case RED_BUILD: doRedBuild();
                break;
            case BLUE_BUILD: doBlueBuild();
                break;
            case BLUE_BLOCKS: doBlueBlocks();
                break;
        }
    }

    private void doRedBlock ()
    {

    }

    private void doRedBuild ()
    {

    }

    private void doBlueBuild ()
    {

    }

    private void doBlueBlocks ()
    {

    }


    public void drive (double distanceCM, double power)
    {
        double startPosition = frontLeft.getCurrentPosition();
        double targetPosition = startPosition + distanceCM*TICKS_PER_CM;
        /*
        telemetry.addData("DistanceCM: ", distanceCM);
        telemetry.addData("TICKS_PER_CENTIMETER: ", TICKS_PER_CM);
        telemetry.addData("Target Ticks: ", distanceCM*TICKS_PER_CM);
        telemetry.addData("Total Ticks: ", startPosition + distanceCM*TICKS_PER_CM);
        telemetry.addData("startPosition", startPosition);
        telemetry.addData("Front Left Position: ", frontLeft.getCurrentPosition());
        telemetry.addData("Target position", targetPosition);
        telemetry.update();
        sleep(5000);
         */

        if (distanceCM > 0)
        {
            while (opModeIsActive() && frontLeft.getCurrentPosition() < targetPosition) {
                frontRight.setPower(power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(power);
                /*
                telemetry.addData("startPosition", startPosition);
                telemetry.addData("Front Left Position: ", frontLeft.getCurrentPosition());
                telemetry.addData("Target position", targetPosition);
                telemetry.();
                 */
            }
        }
        else
        {
            while (opModeIsActive() && frontLeft.getCurrentPosition() > targetPosition)
            {
                frontRight.setPower(-power);
                frontLeft.setPower(-power);
                backRight.setPower(-power);
                backLeft.setPower(-power);
                telemetry.addData("Front Left Position: ", frontLeft.getCurrentPosition());
                telemetry.update();
            }
        }

        ctrlAltDel();
        /*
        telemetry.addData("startPosition", startPosition);
        telemetry.addData("Front Left Position: ", frontLeft.getCurrentPosition());
        telemetry.addData("Target position", targetPosition);
        telemetry.update();
        sleep(5000);
         */
    }

    public void strafe (double distanceCM, boolean right, double power)
    {
        double currentPosition = frontLeft.getCurrentPosition();
        double targetPosition;

        if (right)
        {
            targetPosition = currentPosition + distanceCM*STRAFE_TICKS_PER_CM;

            while (opModeIsActive() && frontLeft.getCurrentPosition() < targetPosition)
            {
                frontRight.setPower(-power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(-power);
                telemetry.addData("Front Left Position: ", frontLeft.getCurrentPosition());
                telemetry.update();
            }
        }
        else
        {
            targetPosition = currentPosition - distanceCM*STRAFE_TICKS_PER_CM;

            while (opModeIsActive() && frontLeft.getCurrentPosition() > targetPosition)
            {
                frontRight.setPower(power);
                frontLeft.setPower(-power);
                backRight.setPower(-power);
                backLeft.setPower(power);
                telemetry.addData("Front Left Position: ", frontLeft.getCurrentPosition());
                telemetry.update();
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

            while (opModeIsActive() && currentAngle > targetAngle)
            {
                frontRight.setPower(powah);
                frontLeft.setPower(-powah);
                backRight.setPower(powah);
                backLeft.setPower(-powah);
                telemetry.addData("secondAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.update();

                currentAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
            }
        }
        else
        {
            targetAngle = currentAngle + angle;

            while (opModeIsActive() && currentAngle < targetAngle)
            {
                frontRight.setPower(-powah);
                frontLeft.setPower(powah);
                backRight.setPower(-powah);
                backLeft.setPower(powah);
                telemetry.addData("secondAngle: ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.update();

                currentAngle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
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

    private void initializeRobot ()
    {
        double intakePower = 0.0;
        double liftPower = 0.0;
        double capstoneArmPosition = 0.0;
        double capstoneHookPosition = 0.0;

        intakeLeft.setPower(intakePower);
        intakeRight.setPower(intakePower);
        lift.setPower(liftPower);
        claw.setPosition(clawOpen);
        clawArm.setPosition(clawArmIn);
        foundGrabber.setPosition(foundationUp);
        capstoneArm.setPosition(capstoneArmPosition);
        capstoneHook.setPosition(capstoneHookPosition);
    }

    private void moveLiftOut ()
    {
        double startPosition = lift.getCurrentPosition();
        double targetPosition = startPosition + 2200;

        // lift up
        while (lift.getCurrentPosition() < targetPosition)
        {
            lift.setPower(0.5);
        }

        claw.setPosition(clawOpen);
        clawArm.setPosition(clawArmOut);

        targetPosition = 0.0;
        // lift down
        while (lift.getCurrentPosition() > targetPosition)
        {
            lift.setPower(-0.5);
        }
        lift.setPower(0.0);
    }

    private void liftLift (double distanceTicks, double power)
    {
        double currentPosition = lift.getCurrentPosition();
        double targetPosition = currentPosition + distanceTicks;
        while (lift.getCurrentPosition() < targetPosition)
        {
            lift.setPower(power);
        }
        lift.setPower(0.0);
    }
}
