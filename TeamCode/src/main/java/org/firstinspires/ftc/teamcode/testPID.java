package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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

@Autonomous(name="testPID", group="Linear OpMode")

public class testPID extends LinearOpMode {

    //Declare OpMode members////////////////////
    private ElapsedTime runtime = new ElapsedTime();        //runtime counter
    private DcMotor frontLeft = null;                       //front left motor
    private DcMotor frontRight = null;                      //front right motor
    private DcMotor backLeft = null;                        //back left motor
    private DcMotor backRight = null;                       //back right motor
    private DcMotor intakeLeft = null;                      //left intake wheel motor
    private DcMotor intakeRight = null;                     //right intake wheel motor
    private DcMotor lift = null;                            //lift motor
    private Servo claw = null;                              //claw servo
    private Servo clawArm = null;                           //claw arm servo
    private Servo foundGrabber = null;                      //foundation grabber servo
    private Servo capstoneArm = null;                       //capstone arm servo
    private Servo capstoneHook = null;                      //capstone hook servo
    private BNO055IMU imu = null;                           //REV Hub internal motion unit
    private ModernRoboticsI2cColorSensor colorSensor = null;//colorSensor

    //Constants for motor/servo positions/////////
    private static final double clawArmIn               = 0.0;  //in
    private static final double clawArmOut              = 1.0;  //in
    private static final double foundationUp            = 0.65; //up
    private static final double foundationDown          = 0.0;  //down
    private static final double clawClosed              = 0.0;  //closed
    private static final double clawOpen                = 1.0;  //open
    private static final double capstoneArmOut          = 1.0;  //out
    private static final double capstoneArmIn           = 0.0;  //in
    private static final double capstoneHookHooked      = 1.0;  //hooked
    private static final double capstoneHookUnHooked    = 0.0;  //unhooked

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

    //set vuForia camera//////////////////////////
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK; //webcam
    private static final boolean PHONE_IS_PORTRAIT = false; //for webcam

    //Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    //We will define some constants and conversions here
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

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //constants
    private final float ENCODER_TICKS_PER_REVOLUTION = 537.6f;
    private final float CIRCUMFERENCE_IN_CM = (float) Math.PI * 10.0f; //diameter is 100mm
    private final float WHEEL_GEAR_REDUCTION = 2f;
    private final float TICKS_PER_CM =  (ENCODER_TICKS_PER_REVOLUTION/CIRCUMFERENCE_IN_CM) * WHEEL_GEAR_REDUCTION; //Experimental Value: 33.69411764705882

    private enum StartPosition {RED_BLOCKS, BLUE_BLOCKS, RED_BUILD, BLUE_BUILD}
    boolean farPark = false;

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
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");

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
        params.loggingEnabled      = true;
        params.loggingTag          = "IMU";
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        turn (90,true, 0.25);
    }

    public void drivePID (double distanceCM, double power)
    {
        double startPosition = frontLeft.getCurrentPosition();
        double targetPosition = startPosition + distanceCM*TICKS_PER_CM;
        double maxPower = power;
        double currentPower = power;
        double error;
        double proportional = 0;

        final double proportion = 1.35;

        while (true && opModeIsActive())
        {
            error = (targetPosition - frontLeft.getCurrentPosition())   //dx = xf - xi
                    /(targetPosition - startPosition);                  //div by total dx
            proportional = proportion*(error*maxPower);
            currentPower = proportional;

            telemetry.addData("Error", error);
            telemetry.update();
            frontRight.setPower(currentPower);
            frontLeft.setPower(currentPower);
            backRight.setPower(currentPower);
            backLeft.setPower(currentPower);

            if (error < 0.001)
            {
                break;
            }
        }
        ctrlAltDel();
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
    }

    public void turn (float angle, boolean CCW, double powah)
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
        else
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

    private void grabBlock ()
    {
        /*
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
         */
    }
}
