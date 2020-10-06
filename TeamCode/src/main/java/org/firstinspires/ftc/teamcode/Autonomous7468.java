package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

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
import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_ARM_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_ARM_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.CAPSTONE_HOOK_HOOKED;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_ARM_IN;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_ARM_OUT;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.RobotConstants.CLAW_OPEN;

@Autonomous(name="Autonomous", group="Linear OpMode")

public class Autonomous7468 extends LinearOpMode {

    //Declare OpMode members////////////////////
    private ElapsedTime runtime = new ElapsedTime();    //runtime counter
    private DcMotor frontLeft = null;                   //front left motor
    private DcMotor frontRight = null;                  //front right motor
    private DcMotor backLeft = null;                    //back left motor
    private DcMotor backRight = null;                   //back right motor
    private DcMotor intakeLeft = null;                  //left intake wheel motor
    private DcMotor intakeRight = null;                 //right intake wheel motor
    private DcMotor lift = null;                        //lift motor
    private Servo claw = null;                          //claw servo
    private Servo clawArm = null;                       //claw arm servo
    private Servo capstoneArm = null;                   //capstone arm servo
    private Servo capstoneHook = null;                  //capstone hook servo
    private Servo blockLifter = null;                   //block lifter servo
    private Servo blockGrabber = null;                  //block grabber servo
    private Servo leftFoundGrabber = null;              //left foundation grabber
    private Servo rightFoundGrabber = null;             //right foundation grabber
    private BNO055IMU imu = null;                       //REV Hub internal motion unit
    private ModernRoboticsI2cColorSensor colorSensor = null;    //color sensor


    //vuForia key/////////////////////////////////
    private static final String VUFORIA_KEY =
            "AWzLk7z/////AAABmddNYTiQD09gpN3oA3v0doxk89BClNkgrwp6vye9ZHmvHIMpmhAWWSZfuIoq6RtEk+lN"
                    + "DQFXTTR98qWs/Q3eKEgjeTZW6hnjMVYMFYZYOqkCqkMSVsn782g6Fc1504xmO42MLdaGmzi9EQ"
                    + "edbZmGYqWZFBt2A84IIT6S6SZTUFDOe0G7Wl/T2zn5CpcY2Cf5GloRtwSbIr4hhUj4fHU4DGf7"
                    + "cc9XbfbbVqjWSP4aM/FzQuJqnTvHsp/yiEvwV0fjfN6NCptgCyfGmHdh4L3NOtXklHFXo0SGV2"
                    + "+/t1td//GBks5RPXDzwE6ODGSndTdkW1dr5U6ZE25niSN3Mz4fJRx4uRtNRCd41ZG9U72qDAWr";

    //vuForia Class Members///////////////////////
    private VuforiaLocalizer vuforia;
    private OpenGLMatrix lastLocation = null;
    List<VuforiaTrackable> allTrackables;
    VuforiaTrackables targetsSkyStone;

    //set vuForia camera//////////////////////////
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK; //webcam
    private static final boolean PHONE_IS_PORTRAIT = false; //for webcam

    //Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    //We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float cmPerInch        = 2.54f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f  * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //constants
    private final float ENCODER_TICKS_PER_REVOLUTION = 383.6f;
    private final float CIRCUMFERENCE_IN_CM = (float) Math.PI * 10.0f; //diameter is 100mm
    private final float WHEEL_GEAR_REDUCTION = 2f;
    private final float TICKS_PER_CM =  (ENCODER_TICKS_PER_REVOLUTION/CIRCUMFERENCE_IN_CM) * WHEEL_GEAR_REDUCTION; //Experimental Value: 33.69411764705882

    //for turning///////////////////////////////////
    private Orientation lastAngles = new Orientation();
    double globalAngle;

    private enum StartPosition {RED_BLOCKS, BLUE_BLOCKS, RED_BUILD, BLUE_BUILD}
    boolean farPark = false;
    boolean moveFoundation = false;
    boolean getBlocks = false;

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
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
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

        //locate drive motors from REV hub setup//
        frontLeft   = hardwareMap.get(DcMotor.class, "frontLeft");  //front left wheel
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight"); //front right wheel
        backLeft    = hardwareMap.get(DcMotor.class, "backLeft");   //back left wheel
        backRight   = hardwareMap.get(DcMotor.class, "backRight");  //back right wheel

        //locate other motors from REV hub setup//
        intakeLeft  = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get (DcMotor.class, "intakeRight");
        lift        = hardwareMap.get(DcMotor.class, "lift");

        //locate servos from REV hub setup//
        claw                = hardwareMap.servo.get("claw");
        clawArm             = hardwareMap.servo.get("clawArm");
        capstoneArm         = hardwareMap.servo.get("capstoneArm");
        capstoneHook        = hardwareMap.servo.get("capstoneHook");
        //blockGrabber        = hardwareMap.servo.get("blockGrabber");
        blockLifter         = hardwareMap.servo.get("blockLifter");
        leftFoundGrabber    = hardwareMap.servo.get("foundGrabberLeft");
        rightFoundGrabber   = hardwareMap.servo.get("foundGrabberRight");

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
        backLeft.setDirection(DcMotor.Direction.FORWARD
        );
        backRight.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        //imu
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        params.calibrationDataFile = "BNO055IMUCalibration.json";
        params.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

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


        telemetry.addData("Ready for parking Position", "");
        telemetry.addData("Far Park", "Up");
        telemetry.addData("Close Park", "Down");
        telemetry.update();

        int runMode = -1;

        while(runMode < 0 && !opModeIsActive())
        {
            if (gamepad1.dpad_up)
            {
                farPark = true;
                telemetry.addData("Park", "Far ");
                telemetry.addData("Select Using", "A");
                telemetry.update();
            }
            else if (gamepad1.dpad_down)
            {
                farPark = false;
                telemetry.addData("Park", "Close ");
                telemetry.addData("Select Using", "A");
                telemetry.update();
            }
            else if (gamepad1.a)
            {
                runMode = 1;
            }
        }

        //until select is let go
        while (gamepad1.a && !opModeIsActive()) {sleep(5);}

        telemetry.addData("Ready for FOUNDATION grabbing", "");
        telemetry.addData("Grab Foundation", "Up");
        telemetry.addData("Don't Grab Foundation", "Down");
        telemetry.update();

        int runMode2 = -1;

        while (runMode2 < 0 && !opModeIsActive())
        {
            if (gamepad1.dpad_up)
            {
                moveFoundation = true;
                telemetry.addData("Move Foundation", "Yes");
                telemetry.addData("Select Using", "B");
                telemetry.update();
            }
            else if (gamepad1.dpad_down)
            {
                moveFoundation = false;
                telemetry.addData("Move Foundation", "No");
                telemetry.addData("Select Using", "B");
                telemetry.update();
            }
            else if (gamepad1.b)
            {
                runMode2 = 1;
            }
        }

        //until select is let go
        while (gamepad1.b && !opModeIsActive()) {sleep(5);}

        telemetry.addData("Park", farPark ? "Far" : "Close");
        telemetry.addData("Move Foundation", moveFoundation ? "Yes" : "No");

        telemetry.addData("Ready for BLOCK grabbing", "");
        telemetry.addData("Grab Blocks", "Up");
        telemetry.addData("Don't Grab Blocks", "Down");
        telemetry.update();

        int runMode3 = -1;
        while (runMode3 < 0)
        {
            if (gamepad1.dpad_up)
            {
                getBlocks = true;
                telemetry.addData("Move Blocks", "Yes");
                telemetry.addData("Select Using", "A");
                telemetry.update();
            }
            else if (gamepad1.dpad_down)
            {
                getBlocks = false;
                telemetry.addData("Move Blocks", "No");
                telemetry.addData("Select Using", "A");
                telemetry.update();
            }
            else if (gamepad1.a)
            {
                runMode3 = 1;
            }
        }

        //until select is let go
        while (gamepad1.a && !opModeIsActive()) {sleep(5);}

        telemetry.addData("Park", farPark ? "Far" : "Close");
        telemetry.addData("Move Foundation", moveFoundation ? "Yes" : "No");
        telemetry.addData("Move Blocks", getBlocks ? "Yes" : "No");

        telemetry.addData("Ready for Position", "");
        telemetry.addData("Up", "Blue Build");
        telemetry.addData("Down", "Blue Blocks");
        telemetry.addData("Y", "Red Build");
        telemetry.addData("A", "Red Blocks");
        telemetry.update();

        runMode = -1;
        StartPosition pos = null;

        while (runMode < 0 && !opModeIsActive())
        {
            if (gamepad1.dpad_up)
            {
                pos = StartPosition.BLUE_BUILD;
                telemetry.addData("Position: ", "Blue Build");
                telemetry.addData("Select Using", "X");
                telemetry.update();
            }
            else if (gamepad1.dpad_down)
            {
                pos = StartPosition.BLUE_BLOCKS;
                telemetry.addData("Position: ", "Blue Blocks");
                telemetry.addData("Select Using", "X");
                telemetry.update();
            }
            else if (gamepad1.y)
            {
                pos = StartPosition.RED_BUILD;
                telemetry.addData("Position: ", "Red Build");
                telemetry.addData("Select Using", "X");
                telemetry.update();
            }
            else if (gamepad1.a)
            {
                pos = StartPosition.RED_BLOCKS;
                telemetry.addData("Position: ", "Red Blocks");
                telemetry.addData("Select Using", "X");
                telemetry.update();
            }
            else if (gamepad1.x)
            {
                runMode = 1;
            }
        }

        telemetry.addData("Park", farPark ? "Far" : "Close");
        telemetry.addData("Move Foundation", moveFoundation ? "Yes" : "No");
        telemetry.addData("Move Blocks", getBlocks ? "Yes" : "No");
        if (pos == StartPosition.BLUE_BLOCKS) {telemetry.addData("Position", "Blue Blocks");}
        else if (pos == StartPosition.BLUE_BUILD) {telemetry.addData("Position", "Blue Build");}
        else if (pos == StartPosition.RED_BLOCKS) {telemetry.addData("Position", "Red Blocks");}
        else if (pos == StartPosition.BLUE_BLOCKS) {telemetry.addData("Position", "Red Build");}
        else {telemetry.addData("Position", "NOT SET");}
        telemetry.addData("Ready to Start!", "");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        Thread setFoundationGrabber = new Thread (new FoundationGrabberSet(lift, leftFoundGrabber, rightFoundGrabber));
        setFoundationGrabber.start();

        if (!(pos == null))
        {
            executeAutonomous(pos);
        }
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
        if (farPark)
        {
            drive(63, 0.35);
            capstoneArm.setPosition(CAPSTONE_ARM_OUT);
            strafe(81.23, true, 0.50);
        }
        else
        {
            drive(3, 0.15);
            drive(15, 0.3);
            capstoneArm.setPosition(CAPSTONE_ARM_OUT);
            drive(-15, 0.3);
            strafe(81.23, true, 0.50);
        }
    }

    private void doRedBuild ()
    {
        if (moveFoundation)
        {
            drive(25, 0.15);
            //go to foundation
            strafe(82.5, false, 0.75);
            //foundGrabber.setPosition(foundationDown);
            sleep(750);
            //move back
            strafe(55*cmPerInch,true, 0.90);
            //turn
            //turn(-35,0.50);
            //set on wall
            strafe(30, true, 0.5);
            //let go
            //foundGrabber.setPosition(foundationUp);
            sleep(250);
            //drive away from foundation
            drive (-29*cmPerInch, 0.5);
            capstoneArm.setPosition(CAPSTONE_ARM_IN);
            if (farPark)
            {
                strafe(25*cmPerInch, false, 0.35);
            }
            //go to line
            driveUntilLine("red", -0.40);
        }
        else
        {
            if (farPark)
            {
                drive(63, 0.15);
                capstoneArm.setPosition(CAPSTONE_ARM_OUT);
            }
            else
            {
                drive(3, 0.15);
                drive(15, 0.3);
                capstoneArm.setPosition(CAPSTONE_ARM_OUT);
                drive(-15, 0.3);
            }
            strafe(81.23, true, 0.5);
        }
    }

    private void doBlueBuild ()
    {
        if (moveFoundation)
        {
            drive(-25, 0.25);
            strafe(82, false, 0.75);
            //foundGrabber.setPosition(foundationDown);
            sleep(750);
            //move back
            strafe(55*cmPerInch,true, 0.90);
            //turn
            //turn(35,0.50);
            //set on wall
            strafe(30, true, 0.5);
            //let go
            //foundGrabber.setPosition(foundationUp);
            sleep(250);
            //drive away from foundation
            drive (27*cmPerInch, 0.5);
            capstoneArm.setPosition(CAPSTONE_ARM_IN);
            if (farPark)
            {
                strafe(20*cmPerInch, false, 0.35);
            }
            //go to line
            driveUntilLine("blue", 0.40);
        }
        else
        {
            if (farPark)
            {
                drive(63, 0.15);
                capstoneArm.setPosition(CAPSTONE_ARM_OUT);
            }
            else
            {
                drive(3, 0.15);
                drive(15, 0.3);
                capstoneArm.setPosition(CAPSTONE_ARM_OUT);
                drive(-15, 0.3);
            }
            strafe(81.23, true, 0.5);
        }
    }

    private void doBlueBlocks ()
    {
        if (farPark)
        {
            drive (63, 0.15);
            capstoneArm.setPosition(CAPSTONE_ARM_OUT);

        }
        else
        {
            drive(3, 0.15);
        }
        strafe(81.23, false, 0.50);
        drive(15, 0.3);
        capstoneArm.setPosition(CAPSTONE_ARM_OUT);
        drive(-15, 0.3);
    }



    public void drive (double distanceCM, double power)
    {
        double startPosition = frontLeft.getCurrentPosition();
        double targetPosition = startPosition + distanceCM*TICKS_PER_CM;
        if (distanceCM > 0)
        {
            while (opModeIsActive() && frontLeft.getCurrentPosition() < targetPosition) {
                frontRight.setPower(power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(power);
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
    }

    public void driveUntilLine (String lineColor, double power)
    {
        if (lineColor.equals("blue"))
        {
            while (opModeIsActive() && colorSensor.blue() < 9)
            {
                frontRight.setPower(power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(power);
            }
        }
        else if (lineColor.equals("red"))
        {
            while (opModeIsActive() && colorSensor.red() < 9)
            {
                frontRight.setPower(power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(power);
            }
        }
        ctrlAltDel();
    }

    public void strafe (double distanceCM, boolean right, double power)
    {
        double currentPosition = frontLeft.getCurrentPosition();
        double targetPosition;

        if (right)
        {
            targetPosition = currentPosition + distanceCM*TICKS_PER_CM;

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
            targetPosition = currentPosition - distanceCM*TICKS_PER_CM;

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

    public void strafeUntilLine (String lineColor, double power, boolean right)
    {
        if (right)
        {
            if (lineColor.equals("blue"))
            {
                while (opModeIsActive() && colorSensor.blue() < 9)
                {
                    frontRight.setPower(-power);
                    frontLeft.setPower(power);
                    backRight.setPower(power);
                    backLeft.setPower(-power);
                }
            }
            else if (lineColor.equals("red"))
            {
                while (opModeIsActive() && colorSensor.red() < 9)
                {
                    frontRight.setPower(-power);
                    frontLeft.setPower(power);
                    backRight.setPower(power);
                    backLeft.setPower(-power);
                }
            }
        }
        else
        {
            if (lineColor.equals("blue"))
            {
                while (opModeIsActive() && colorSensor.blue() < 9)
                {
                    frontRight.setPower(power);
                    frontLeft.setPower(-power);
                    backRight.setPower(-power);
                    backLeft.setPower(power);
                }
            }
            else if (lineColor.equals("red"))
            {
                while (opModeIsActive() && colorSensor.red() < 9)
                {
                    frontRight.setPower(power);
                    frontLeft.setPower(-power);
                    backRight.setPower(-power);
                    backLeft.setPower(power);
                }
            }
        }

        ctrlAltDel();
    }


    public void turn (double angleCCW, double powah)
    {
        //#define powah power;

        resetAngle();
        double startAngle = getAngle();
        double targetAngle = startAngle + angleCCW;
        double deltaAngle = targetAngle - startAngle;
        double currentAngle = startAngle;
        if (deltaAngle > 0)
        {
            while (currentAngle < targetAngle && opModeIsActive())
            {
                frontRight.setPower(-powah);
                frontLeft.setPower(powah);
                backRight.setPower(-powah);
                backLeft.setPower(powah);
                currentAngle = getAngle();
                telemetry.addData("Current Angle", currentAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.update();
            }

            ctrlAltDel();
        }
        else if (deltaAngle < 0)
        {
            powah *= -1;
            while (currentAngle > targetAngle && opModeIsActive())
            {
                frontRight.setPower(-powah);
                frontLeft.setPower(powah);
                backRight.setPower(-powah);
                backLeft.setPower(powah);
                currentAngle = getAngle();
                telemetry.addData("Current Angle", currentAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.update();
            }

            ctrlAltDel();
        }
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
        double capstoneArmPosition = CAPSTONE_ARM_IN;
        double capstoneHookPosition = CAPSTONE_HOOK_HOOKED;

        intakeLeft.setPower(intakePower);
        intakeRight.setPower(intakePower);
        lift.setPower(liftPower);
        claw.setPosition(CLAW_OPEN);
        clawArm.setPosition(CLAW_ARM_IN);
        //foundGrabber.setPosition(foundationUp);
        capstoneArm.setPosition(capstoneArmPosition);
        capstoneHook.setPosition(capstoneHookPosition);
    }

    private void moveLiftOut ()
    {
        double startPosition = lift.getCurrentPosition();
        double targetPosition = startPosition + 2200;

        // lift up
        while (opModeIsActive() && lift.getCurrentPosition() < targetPosition)
        {
            lift.setPower(0.5);
        }

        claw.setPosition(CLAW_OPEN);
        clawArm.setPosition(CLAW_ARM_OUT);

        targetPosition = 0.0;
        // lift down
        while (opModeIsActive() && lift.getCurrentPosition() > targetPosition)
        {
            lift.setPower(-0.5);
        }
        lift.setPower(0.0);
    }

    private void liftLift (double distanceTicks, double power)
    {
        double currentPosition = lift.getCurrentPosition();
        double targetPosition = currentPosition + distanceTicks;
        while (opModeIsActive() && lift.getCurrentPosition() < targetPosition)
        {
            lift.setPower(power);
        }
        lift.setPower(0.0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private boolean isTargetVisible ()
    {

        double startTime = runtime.time();

        targetsSkyStone.activate();

        while (!isStopRequested() && runtime.time() < startTime + 2.5)
        {
            telemetry.addData("running isTargetVisible()", "");
            telemetry.update();
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
        }

        telemetry.addData("target is", targetVisible ? "visible" : "not visible");
        telemetry.update();

        targetsSkyStone.deactivate();

        return targetVisible;
    }

    private void grabBlock ()
    {
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
        claw.setPosition(CLAW_CLOSED);
        sleep(1000);
        liftLift(250, 0.25);
        drive(-30, 0.5);
        strafe(30, true, 0.5);

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }
}
