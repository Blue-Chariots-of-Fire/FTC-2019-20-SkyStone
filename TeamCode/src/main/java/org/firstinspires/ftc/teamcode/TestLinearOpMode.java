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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


@TeleOp(name="TestTeleOpMode", group="Linear Opmode")

public class TestLinearOpMode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor intakeLeft = null;
    private DcMotor intakeRight = null;

    /*
    //skystone names for object detection
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    //object detection and localization stuff
    private static final String VUFORIA_KEY =
            "AWzLk7z/////AAABmddNYTiQD09gpN3oA3v0doxk89BClNkgrwp6vye9ZHmvHIMpmhAWWSZfuIoq6RtEk+lN"
                    + "DQFXTTR98qWs/Q3eKEgjeTZW6hnjMVYMFYZYOqkCqkMSVsn782g6Fc1504xmO42MLdaGmzi9EQ"
                    + "edbZmGYqWZFBt2A84IIT6S6SZTUFDOe0G7Wl/T2zn5CpcY2Cf5GloRtwSbIr4hhUj4fHU4DGf7"
                    + "cc9XbfbbVqjWSP4aM/FzQuJqnTvHsp/yiEvwV0fjfN6NCptgCyfGmHdh4L3NOtXklHFXo0SGV2"
                    + "+/t1td//GBks5RPXDzwE6ODGSndTdkW1dr5U6ZE25niSN3Mz4fJRx4uRtNRCd41ZG9U72qDAWr";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    */

    //slowMode boolean
    private boolean slowMode = false;
    private boolean intake = false;
    private boolean intakeReverse = false;

    // Setup a variable for each drive wheel
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double intakePower;

    //variable for the controllers
    private double turn;
    private double drive;
    private double strafe;


    public void runOpMode()
    {
        //initVuforia(); //initializes vuforia

        /*
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        */

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get (DcMotor.class, "intakeRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            checkIntake();
            checkSlowMode();
            move();
            telemetry();
        }
    }

    //checks if the robot is on slow mode and changes the mode
    private void checkSlowMode()
    {
        if (gamepad1.a)
        {
            if (slowMode == true)
            {
                slowMode = false;
            }
            else
            {
                slowMode = true;
            }
        }
    }

    private void checkIntake ()
    {
        if (gamepad1.b)
        {
            if (intakeReverse == true)
            {
                intakeReverse = false;
            }

            if (intake == true)
            {
                intake = false;
            }
            else
            {
                intake = true;
            }
        }

        if (gamepad1.y)
        {
            if (intake == true)
            {
                intake = false;
            }

            if (intakeReverse == true)
            {
                intakeReverse = false;
            }
            else
            {
                intakeReverse = true;
            }
        }
    }

    //moves the robot for one hardware cycle
    private void move ()
    {
        drive = -gamepad1.left_stick_y;
        turn  =  -gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        if (slowMode)
        {
            frontLeftPower = Range.clip((drive-turn+strafe), -1.0, 1.0);
            frontRightPower = Range.clip((drive+turn-strafe), -1.0, 1.0);
            backLeftPower = Range.clip((drive-turn-strafe), -1.0, 1.0);
            backRightPower = Range.clip((drive+turn+strafe), -1.0, 1.0);
        }
        if (!slowMode)
        {
            frontLeftPower = (drive-turn+strafe)/3;
            frontRightPower = (drive+turn-strafe)/3;
            backLeftPower = (drive-turn-strafe)/3;
            backRightPower = (drive+turn+strafe)/3;
        }

        if (intake)
        {
            intakePower = 1.0;
        }
        else
        {
            intakePower = 0.0;
        }

        if (intakeReverse)
        {
            intakePower = -1.0;
        }
        else
        {
            intakePower = 0.0;
        }

        // Send calculated power to motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
        intakeLeft.setPower(intakePower);
        intakeRight.setPower(intakePower);
    }

    //adds telemetry info to the driver station
    private void telemetry ()
    {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower);
        telemetry.update();
    }

    /*
    //initializes vuforia localization engine
    private void initVuforia() {
        //
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        //
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    //initializes tensorflow object detection engine
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    */

}
