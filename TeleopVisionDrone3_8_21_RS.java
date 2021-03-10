/* Copyright (c) 2019 FIRST. All rights reserved.
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


/*
/**
 * This 2020-2021 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the ULTIMATE GOAL FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * There are a total of five image targets for the ULTIMATE GOAL game.
 * Three of the targets are placed in the center of the Red Alliance, Audience (Front),
 * and Blue Alliance perimeter walls.
 * Two additional targets are placed on the perimeter wall, one in front of each Tower Goal.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ultimategoal/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

/**
 * {@link SensorBNO055IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision - Imports  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV */
        
/* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision - Imports END VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV */

@TeleOp ( name = "TeleopVisionDrone3_8_21_RS" , group = "" )
public class TeleopVisionDrone3_8_21_RS extends LinearOpMode {

    /* Define the motors and servors being used
     */
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor conveyorMotor;
    private DcMotor launcherMotor;
    private DcMotor wobbleGoalMotor;
    private CRServo intakeServo;
    private Servo wobbleGoalClampServo;
    private DcMotor inclineMotor;

    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision variable declarations    VVVVVVVVVVVVVVVVVVVV */
    // Use for time duration move commands
    private ElapsedTime runtime = new ElapsedTime();
    double CountsPerInch;
    double CountsPerDegree;
    double InchesPerRev;
    double TargetDistance;
    double TargetTime;
    double CountsPerDegreePlatform;
    double platformHeight;
    double oppositePlatform;
    double xLocationPlatform;
    double adjacentPlatform;
    double hypotenusePlatform;
    double idealAimAnglePlatform;
    double targetPlatformCount;
    int targetPlatformCount1;
    int encoderIncline;
    int InclineCount;

    //  DECLARE VARIABLES FOR TRACKING
    private long      e1, e2, e3, e4; // Encoder Values
    long      e1LastEncPosition, e2LastEncPosition, e3LastEncPosition, e4LastEncPosition;
    long      e1Change, e2Change, e3Change, e4Change;
    long      avgEncoder;
    private double    xPosition = 0.0, yPosition = 35.5;
    int LeftDrvMtrCnt_C;
    int RightDrvMtrCnt_C;
    int LeftDrvMtrFntCnt_C;
    int RightDrvMtrFntCnt_C;
    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision variable declarations END   VVVVVVVVVVVVVVVVVVVV */

    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision Comments   VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV */
    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //

    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision Comments END  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV */


    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision Settings   VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  */
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision Settings END VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  */


    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision Comments   VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV */
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     *
     *
     VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV Vision Comments END   VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV*/


    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision Vuforia Variable Declarations  VVVVVVVVVVVVVVVVVVVVVVVVVVVV */
    private static final String VUFORIA_KEY =
            "AZkDD8L/////AAABmWP4MyhQS0VWhSXaK524cRw06Qm9rCF1ZMtZ9HgrijqYPfhSXcVDodHFgrvtOpReoQ/Rib1AhWR0kAqqP+xSA6xRRjVZU3YWZ1xYMZGK7RfiB4wbdFNdbeymz3kLhcNdnUSEipRT/hgm32V/iXXy8sDfT0GqZR4P2yvCd7w+RREIGSHWDf8ZqfrS1e9KY9RBUvM6VjeJsJcucSjhxISSmq/hcU5PZ5bnWSNcDShTAwWJyWGGC61/utWoZ2l8E7dCPT3I0Ia4mo4CEkwpTXAN0iAH2QF2H5jH8QuWui8gHtVEz/st2SuLpUBK9YU4WDad/kv5nDFQPeFrhfh7ZRM43JA02pNDT99Kx+CYZW8DODDu";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    //----------------------------------------------------------------------------------------------
    // State IMU Logic
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision Vuforia Variable Declarations END VVVVVVVVVVVVVVVVVVVVVVVVVVVV */



    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        /* values for joystick position and how much movement is required for action to occur
         */
        double threshold = 0.2;
        double RightStickX;
        double LeftStickX;
        double LeftStickY;
        double wobble_power;
        double Wobble;
        double clampposition;
        Boolean Clamp;
        Boolean Conveyor;
        Boolean pressedB = false; // this variable at the class level, at the top for launcher
        Boolean droneModeOn = false; // this variable at the class level, at the top for Drone mode

        /* Map the hardware being used
         */
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        launcherMotor = hardwareMap.get(DcMotor.class, "launcherMotor");
        wobbleGoalMotor = hardwareMap.get(DcMotor.class, "wobbleGoalMotor");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        wobbleGoalClampServo = hardwareMap.get(Servo.class, "wobbleGoalClampServo");
        inclineMotor = hardwareMap.get(DcMotor.class, "inclineMotor");

        /*    VVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision Parameter Configurations   VVVVVVVVVVVVVVVVVVVVVVVVVV
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            // Phone is  angled down 16 degrees (fixed position) to see ring location from start position of robot for autonomous mode
            phoneYRotate = -74;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 5.9f * mmPerInch;   // eg: Camera is 6 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 12.5f * mmPerInch;   // eg: Camera is 12.5 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        /*    VVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision Parameter Configurations END  VVVVVVVVVVVVVVVVVVVVVVVVVV


        /*  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision IMU Variable assignments and setup   VVVVVVVVVVVVVVVVVVVVVVVVVVV */
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IMUparameters.loggingEnabled      = true;
        IMUparameters.loggingTag          = "IMU";
        IMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);

        // Set up our telemetry dashboard
        composeTelemetry();
        /*  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision IMU Variable assignments and setup END   VVVVVVVVVVVVVVVVVVVVVVVVVVV */


        waitForStart();

        // Put initialization blocks here.
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Return motors to mode for TeleOp operation
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        wobbleGoalMotor.setDirection(DcMotor.Direction.REVERSE);
        conveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleGoalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inclineMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleGoalClampServo.setPosition(0.5);

        /* VVVVVVVVVVVVVVVVVVVVVVVV   Vision IMU start for heading info  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV */
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        /* VVVVVVVVVVVVVVVVVVVVVVVV   Vision IMU start for heading info END  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV */

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        /* VVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision related variable declaration   VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV */
        float rotationError = 0.0f;
        float launchLineDistance = 0.0f;
        float rotationTime = 0.0f;
        float reverseTime = 0.0f;
        float yLocation = 0.0f;
        float xLocation = 0.0f;
        float idealAimAngle = 0.0f;
        float currentAimAngle = 0.0f;
        float correctiveAngleAdj = 0.0f;
        float opposite = 0.0f;
        float adjacent = 0.0f;
        float hypotenuse = 0.0f;
        double droneModeLeftStickX = 1.0d;
        double droneModeLeftStickY = 1.0d;
        double droneModeAdjLFRR = 1.0d;
        double droneModeAdjRFLR = 1.0d;

        float headingRobot = 0.0f;

        /* VVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision related variable declaration END  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV */


        /* VVVVVVVVVVVVVVVVVVVV  Vision - Activate Vuforia target tracking      VVVVVVVVVVVVVVVVVVVVVV  */
        targetsUltimateGoal.activate();

        CountsPerDegreePlatform = 4.0;
        platformHeight = 10.0;
        xLocationPlatform = 8.0;

        // Calculate actual aim angle
        oppositePlatform = (35.875 - platformHeight);
        adjacentPlatform = (72.0 - xLocationPlatform);
        hypotenusePlatform = (sqrt(adjacentPlatform * adjacentPlatform + oppositePlatform * oppositePlatform));
        idealAimAnglePlatform = toDegrees(asin(oppositePlatform / hypotenusePlatform));
        telemetry.addData("IdealAimAngle", "Angle: %2.5f", idealAimAnglePlatform);
        sleep(2000);
        targetPlatformCount = CountsPerDegreePlatform * idealAimAnglePlatform;
        targetPlatformCount1 = (int) targetPlatformCount;

        // Read current encoder position
        encoderIncline = inclineMotor.getCurrentPosition();
        InclineCount = (targetPlatformCount1 + encoderIncline);


        //waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                {

                    RightStickX = 0.75 * gamepad1.right_stick_x;
                    LeftStickX = 0.75 * gamepad1.left_stick_x;
                    LeftStickY = 0.75 * gamepad1.left_stick_y;

                    //telemetry.addData("Target Power", LeftStickY);
                    //telemetry.addData("Target Power", LeftStickX);
                    //telemetry.update();
                    
                    
                    /*  VVVVVVVVVVVVVVVVVVV  Vision - update IMU for heading tracking   VVVVVVVVVVVVVVVVVVVVVVV */
                    // IMU Function
                    telemetry.update();
                    headingRobot = angles.firstAngle;
                    
                    // Check for drone mode
                    if (gamepad1.x) {
                        droneModeOn = !droneModeOn;
                        sleep (1000);
                    }

                    if (droneModeOn) {
                    
                        // Determine if robot is heading at an angle
                        if ((headingRobot <= 0.0) && (headingRobot >= -45.0)) {
                            if (abs(LeftStickY) >= abs(LeftStickX)) {
                                droneModeAdjLFRR = (1.0 + (headingRobot / 45.0));
                                droneModeAdjRFLR = 1.0d;
                            } else {
                                droneModeAdjLFRR = 1.0d;
                                droneModeAdjRFLR = (1.0 + (headingRobot / 45.0));
                            }
                        } else if ((headingRobot > 0.0) && (headingRobot <= 45.0)) {
                            if (abs(LeftStickY) >= abs(LeftStickX)) {
                                droneModeAdjLFRR = 1.0d;
                                droneModeAdjRFLR = (1.0 - (headingRobot / 45.0));
                            } else {
                                droneModeAdjLFRR = (1.0 - (headingRobot / 45.0));
                                droneModeAdjRFLR = 1.0d;
                            }
                        } else if ((headingRobot <= -45.0) && (headingRobot > -90.0 )) {
                            if (abs(LeftStickY) >= abs(LeftStickX)) {
                                droneModeAdjLFRR = (headingRobot + 45.0) / 45.0;
                                droneModeAdjRFLR = 1.0d;
                            } else {
                                droneModeAdjLFRR = 1.0d;
                                droneModeAdjRFLR = (headingRobot + 45.0) / 45.0;
                            }
                        } else if ((headingRobot >= 45.0) && (headingRobot < 90.0 )) {
                            if (abs(LeftStickY) >= abs(LeftStickX)) {
                                droneModeAdjLFRR = 1.0d;
                                droneModeAdjRFLR = -(headingRobot - 45.0) / 45.0;
                            } else {
                                droneModeAdjLFRR = -(headingRobot - 45.0) / 45.0;
                                droneModeAdjRFLR = 1.0d;
                            }
                        } else if ((headingRobot <= -90.0) && (headingRobot > -135.0 )) {
                            if (abs(LeftStickY) >= abs(LeftStickX)) {
                                droneModeAdjLFRR = -1.0d;
                                droneModeAdjRFLR = (1.0 + (headingRobot + 90.0) / 45.0);
                            } else {
                                droneModeAdjLFRR = (1.0 + (headingRobot + 90.0) / 45.0);
                                droneModeAdjRFLR = -1.0d;
                            }
                        } else if ((headingRobot >= 90.0) && (headingRobot < 135.0 )) {
                            if (abs(LeftStickY) >= abs(LeftStickX)) {
                                droneModeAdjLFRR = (1.0 - (headingRobot - 90.0) / 45.0);
                                droneModeAdjRFLR = -1.0d;
                            } else {
                                droneModeAdjLFRR = -1.0d;
                                droneModeAdjRFLR = (1.0 - (headingRobot - 90.0) / 45.0);
                            }
                        } else if ((headingRobot <= -135.0) && (headingRobot >= -180.0 )) {
                            if (abs(LeftStickY) >= abs(LeftStickX)) {
                                droneModeAdjLFRR = -1.0d;
                                droneModeAdjRFLR = (headingRobot + 135.0) / 45.0;
                            } else {
                                droneModeAdjLFRR = (headingRobot + 135.0) / 45.0;
                                droneModeAdjRFLR = -1.0d;
                            }
                        } else if ((headingRobot >= 135.0) && (headingRobot <= 180.0 )) {
                            if (abs(LeftStickY) >= abs(LeftStickX)) {
                                droneModeAdjLFRR = -(headingRobot - 135.0) / 45.0;
                                droneModeAdjRFLR = -1.0d;
                            } else {
                                droneModeAdjLFRR = -1.0d;
                                droneModeAdjRFLR = -(headingRobot - 135.0) / 45.0;
                            }
                        } 

                    } else {
                        droneModeAdjLFRR = 1.0d;
                        droneModeAdjRFLR = 1.0d;
                    }
                    
                    /* code for driving the Mecanum wheels
                     */
                    if (abs(gamepad1.left_stick_y) > threshold || abs(gamepad1.left_stick_x) > threshold) {
                        //[rightFront] = (gamepad1.right_stick_y - gamepad1.right_stick_x);
                        rightFront.setPower((-LeftStickY - LeftStickX) * droneModeAdjRFLR);
                        //rightFront.setPower(-droneModeLeftStickY - droneModeLeftStickX);
                        //[leftFront] = (-gamepad1.right_stick_y - gamepad1.right_stick_x);
                        leftFront.setPower((LeftStickY  - LeftStickX) * droneModeAdjLFRR );
                        //leftFront.setPower(droneModeLeftStickY  - droneModeLeftStickX );
                        //[rightRear] = (-gamepad1.right_stick_y - gamepad1.right_stick_x);
                        rightRear.setPower((LeftStickY - LeftStickX) * droneModeAdjLFRR);
                        //rightRear.setPower(droneModeLeftStickY - droneModeLeftStickX);
                        //DcMotor[leftRear] = (gamepad1.right_stick_y - gamepad1.right_stick_x);
                        leftRear.setPower((-LeftStickY - LeftStickX) * droneModeAdjRFLR);
                        //leftRear.setPower(-droneModeLeftStickY - droneModeLeftStickX);
                    } else if (abs(gamepad1.left_stick_y) < threshold || abs(gamepad1.left_stick_x) < threshold) {
                        //[rightFront] = (gamepad1.right_stick_y - gamepad1.right_stick_x);
                        rightFront.setPower(0);
                        //[leftFront] = (-gamepad1.right_stick_y - gamepad1.right_stick_x);
                        leftFront.setPower(0);
                        //[rightRear] = (-gamepad1.right_stick_y - gamepad1.right_stick_x);
                        rightRear.setPower(0);
                        //DcMotor[leftRear] = (gamepad1.right_stick_y - gamepad1.right_stick_x);
                        leftRear.setPower(0);
                    }
                    if (abs(gamepad1.right_stick_x) > threshold) {
                    /*
                    rotate using the Mecanum wheels
                    */
                        rightFront.setPower(-RightStickX);
                        leftFront.setPower(-RightStickX);
                        rightRear.setPower(RightStickX);
                        leftRear.setPower(RightStickX);
                    }
                    /* gamepad command to run the conveyor motor
                     */
                    if (gamepad2.dpad_up) {
                        conveyorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                        conveyorMotor.setPower(1);
                    } else if (gamepad2.dpad_down) {
                        conveyorMotor.setPower(0);

                            Conveyor = gamepad2.dpad_up;
                            telemetry.addData("Conveyor Motor", Conveyor);
                    }

                    /* Code for Intake lift
                     */
                    // Gamepad 2: A and Y buttons for up/down a=down y=up

                    if (gamepad2.a) {
                            // 4a. up
                            intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
                            intakeServo.setPower(1);
                    } else if (gamepad2.y) {
                            // 4b. down
                            intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
                            intakeServo.setPower(1);
                    } else intakeServo.setPower(0);

                    }


                /* Code for the Launcher motor
                 */

                    if (gamepad2.b) {
                    pressedB = !pressedB; //this will revert the value: if true -> false, if false -> true
                    }

                    if (pressedB) {
                    launcherMotor.setPower(1);
                    } else {
                    launcherMotor.setPower(0);
                    }

                /* gamepad command to run the Wobble Goal motor for raising/lowering Wobble goal
                 */
                // Gamepad 2: Left Joystick: Wobble goal motor up/down
                    wobble_power = 0.75 * gamepad2.left_stick_y;
                    wobbleGoalMotor.setPower(wobble_power);

                    Wobble = gamepad2.left_stick_y;
                    telemetry.addData("Wobble Lift", Wobble);


                /* Code for wobble goal clamp
                 */
                // Gamepad 2: Bumpers: Wobble clamp open/close


                    if (gamepad2.left_bumper) {
                        wobbleGoalClampServo.setPosition(.75);
                    }
                    if (gamepad2.right_bumper) {
                        wobbleGoalClampServo.setPosition(-.75);
                    }
                    Clamp = gamepad2.right_bumper;
                    telemetry.addData("Clamp", Clamp);

                    if (gamepad1.left_bumper) {
                        inclineMotor.setTargetPosition(InclineCount);
                        inclineMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        inclineMotor.setPower(0.5);
                        inclineMotor.setPower(0);
                    }

                    if (gamepad1.right_bumper) {
                    inclineMotor.setTargetPosition(-InclineCount);
                    inclineMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    inclineMotor.setPower(0.5);
                    inclineMotor.setPower(0);
                    }

                /*  VVVVVVVVVVVVVVVVVVV  Vision - update IMU for heading tracking  END  VVVVVVVVVVVVVVVVVVVVVVV */


                /* VVVVVVVVVVVVVVVVVVVVVV  Vision-related tracking - UNDER DEVELOPMENT  VVVVVVVVVVVVVVVVVVVVVVV
                // Save last encoder position to calculate change
                e1LastEncPosition = e1;
                e2LastEncPosition = e2;
                e3LastEncPosition = e3;
                e4LastEncPosition = e4;
                */

                // Read new encoder data
                    e1 = leftRear.getCurrentPosition();
                    e2 = leftFront.getCurrentPosition();
                    e3 = rightRear.getCurrentPosition();
                    e4 = rightFront.getCurrentPosition();

                /*
                // telemetry.addData("SpeedLeftR", v3);
                // telemetry.addData("SpeedRightF", v2);
                // telemetry.addData("SpeedLeftF", v1);
                // telemetry.addData("IdealAimAngle", "Angle: %2.5f", idealAimAngle);

                // Calculate encoder position changes since last scan
                e1Change = e1 - e1LastEncPosition;
                e2Change = e2 - e2LastEncPosition;
                e3Change = e3 = e3LastEncPosition;
                e4Change = e4 - e4LastEncPosition;

                // Now determine based on position changes for each robot wheel
                // Must also take heading of robot into account to determine direction traveled
                // If leftFront rightRear wheel are both moving backwards-ONLY, update XY position based on counts and proportion of encoder changes

                avgEncoder = (long) ((e1Change + e2Change +e3Change + e4Change) / 4.0);

                // Increase x position based on encoder counts.
                xPosition = (float) (xPosition + (avgEncoder / 122.63));
                telemetry.addData("xPosition", " %2.5f", xPosition                    );

                VVVVVVVVVVVVVVVVVVVVVV  Vision-related tracking - UNDER DEVELOPMENT END VVVVVVVVVVVVVVVVVVVVVVV */


                /*  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVV   Vision code so start scanning for targets   VVVVVVVVVVVVVVVVVVVVVVVV   */
                // check all the trackable targets to see which one (if any) is visible.
                    targetVisible = false;
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }
                /*  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVV   Vision code so start scanning for targets END  VVVVVVVVVVVVVVVVVVVVVVVV   */

                /* VVVVVVVVVVVVVVVVVVVVVV  Vision to process visible targets VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  */
                // Provide feedback as to where the robot is located (if we know).
                    if (targetVisible) {
                        // express position (translation) of robot in inches.
                        VectorF translation = lastLocation.getTranslation();
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                        rotationError = rotation.thirdAngle;
                        launchLineDistance = (float) ((translation.get(0) / mmPerInch));
                        yLocation = translation.get(1) / mmPerInch;
                    }
                    else {
                        telemetry.addData("Visible Target", "none");
                        rotationError = 0.0f;
                        launchLineDistance = 0.0f;
                    }
                    // Print encoder counts on driver station
                    telemetry.addData("Position leftRear", e1);
                    telemetry.addData("Position leftFront", e2);
                    telemetry.addData("Position rightRear", e3);
                    telemetry.addData("Position rightFront", e4);

                    telemetry.update();
                /* VVVVVVVVVVVVVVVVVVVVVV  Vision to process visible targets END VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  */


                /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision application code    VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  */
                /* START OF APPLICATION CODE
                This example is for BLUE Alliance mat setup
                Show key variables on driver station phone for debug */
                //telemetry.addData("rotationError", " = %.1f", rotationError);
                //telemetry.addData("launchLineDistance", " = %.1f", launchLineDistance);

                /* Driver gamepad 1 left trigger initiates Step 1 of AutoAim -  which is to square robot to wall and backup robot until it is behind launch line
                Please note that all example robot moves are timer-based and could be switched to
                encoder-based moves for additional accuracy */

                // Check if driver requests AutoAim (Step 1) AND robot is located between launch line and tower goals
                    if ( gamepad1.left_trigger == 1 && launchLineDistance > 0.0) {

                        // Negative rotation error indicates robot is currently pointed right of center and requires auto left rotation to square up to tower goal
                        if (rotationError < 0.0) {

                            // Multiply rotation error by 0.015 seconds of rotation time per rotation degree (constant determined by testing)
                            rotationTime = (float) (-rotationError * 0.015);
                            TargetTime = rotationTime;
                            TargetDistance = -rotationError;
                            RotateLeft();

                            // Positive rotation error indicates robot is currently pointed left of center and requires auto right rotation to square up to tower goal
                        } else if (rotationError > 0.0) {

                            // Multiply rotation error by 0.015 seconds of rotation time per rotation degree (constant determined by testing)
                            rotationTime = (float) (rotationError * 0.015);
                            TargetTime = rotationTime;
                            TargetDistance = rotationError;
                            RotateRight();
                        }

                        // Auto drive backwards far enough to get behind the launch line

                        TargetDistance = launchLineDistance;
                        DriveBackwards();
                    }

                    /* Driver gampad 1 right trigger initiates Step 2 of AutoAim -  which is rotating the robot to aim at the BLUE tower goal target based on Y-coordinate location */
                    /* Check if driver requests AutoAim (Step 2)
                    ONLY perform the aim rotation function if robot is behind the launch line */
                    if (gamepad1.right_trigger == 1) {

                        if (targetVisible) {


                        /* Make sure robot is behind the launch line
                        It is assumed that robot is just behind the launch line (based on performing Step 1 of AutoAim) */
                            if (launchLineDistance < 4.0) {

                                // Calculate ideal aim angle based on yLocation of robot
                            /* IMPROVED ANGLE CALCULATION
                            1.  Calculate length of adjacent, opposite, and hypotenuse of right triangle sides for robot aim (see Vision Doc for diagram)
                            2.  To determine actual aim angle, calculate the arcsin of the ratio of the Y robot field coordinate from center of the hypotenuse
                            3.  Multiply this result by 0.02269 seconds of rotation time per rotation degree (constant determined by testing)
                            4.  Auto rotate proper direction based on sign of rotationTime */

                                // Calculate actual aim angle
                                opposite = (float) (yLocation - 35.375);
                                adjacent = (float) (72.0 - xLocation);
                                hypotenuse = (float) (sqrt(adjacent * adjacent + opposite * opposite));
                                idealAimAngle = (float) toDegrees(asin(opposite / hypotenuse));
                                telemetry.addData("IdealAimAngle", "Angle: %2.5f", idealAimAngle);
                                telemetry.update();
                                // sleep(2000);

                                currentAimAngle = rotationError;
                                correctiveAngleAdj = -idealAimAngle - currentAimAngle;
                            }


                            rotationTime = (float) (correctiveAngleAdj * 0.02269);

                            if (correctiveAngleAdj < -0.5) {
                                TargetTime = -rotationTime;
                                TargetDistance = -correctiveAngleAdj;
                                RotateRight();

                            } else if (correctiveAngleAdj > 0.5) {
                                TargetTime = rotationTime;
                                TargetDistance = correctiveAngleAdj;
                                RotateLeft();
                            }

                        } else {

                        /* If there is no target visible, use IMU rotation data to point robot directly (square) towards tower goal wall*/
                        correctiveAngleAdj = -angles.firstAngle;
                        rotationTime = (float) (correctiveAngleAdj * 0.02269);

                        // rough rotation adjustment
                        if (correctiveAngleAdj < -0.5) {
                            TargetTime = -rotationTime;
                            TargetDistance = -correctiveAngleAdj;
                            RotateRight();


                        } else if (correctiveAngleAdj > 0.5) {
                            TargetTime = rotationTime;
                            TargetDistance = correctiveAngleAdj;
                            RotateLeft();
                        }

                        // fine rotation adjustment
                        // read IMU angle again
                        telemetry.update();
                        correctiveAngleAdj = -angles.firstAngle;
                        if (correctiveAngleAdj < -0.5) {
                            TargetTime = -rotationTime;
                            TargetDistance = -correctiveAngleAdj;
                            RotateRight();

                        } else if (correctiveAngleAdj > 0.5) {
                            TargetTime = rotationTime;
                            TargetDistance = correctiveAngleAdj;
                            RotateLeft();
                        }

                    }

                }
                /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision application code END   VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  */

            }

            /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision command to stop the vision functions   VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV */
            // Disable Tracking when we are done;
            targetsUltimateGoal.deactivate();

            /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision command to stop the vision functions END  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV */


        }
    }

    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision methods to move the robot set amounts based on vision info    VVVVVVVVVVVVVV */
    private void RotateLeft() {
        // Encoder counts per degree of rotation
        CountsPerDegree = 31.0;

        // Read current encoder position
        e1 = leftFront.getCurrentPosition();
        e2 = rightFront.getCurrentPosition();
        e3 = leftRear.getCurrentPosition();
        e4 = rightRear.getCurrentPosition();

        // Calculate position for rotation
        LeftDrvMtrFntCnt_C = (int) (TargetDistance * CountsPerDegree + e1);
        RightDrvMtrFntCnt_C = (int) (TargetDistance * CountsPerDegree + e2);
        LeftDrvMtrCnt_C = (int) (-TargetDistance * CountsPerDegree + e3);
        RightDrvMtrCnt_C = (int) (-TargetDistance * CountsPerDegree + e4);

        // Set target for rotation amount
        leftFront.setTargetPosition(LeftDrvMtrFntCnt_C);
        rightFront.setTargetPosition(RightDrvMtrFntCnt_C);
        leftRear.setTargetPosition(LeftDrvMtrCnt_C);
        rightRear.setTargetPosition(RightDrvMtrCnt_C);

        // Set motor modes
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power level for motors
        leftFront.setPower(0.37);
        rightFront.setPower(0.37);
        leftRear.setPower(0.37);
        rightRear.setPower(0.37);

        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
        }

        // Turn motors off
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

        // Return motors to mode for TeleOp operation
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void RotateRight() {
        // Encoder counts per degree of rotation
        CountsPerDegree = 31.0;


        // Read current encoder position
        e1 = leftFront.getCurrentPosition();
        e2 = rightFront.getCurrentPosition();
        e3 = leftRear.getCurrentPosition();
        e4 = rightRear.getCurrentPosition();

        // Calculate position for rotation
        LeftDrvMtrFntCnt_C = (int) (-TargetDistance * CountsPerDegree + e1);
        RightDrvMtrFntCnt_C = (int) (-TargetDistance * CountsPerDegree + e2);
        LeftDrvMtrCnt_C = (int) (TargetDistance * CountsPerDegree + e3);
        RightDrvMtrCnt_C = (int) (TargetDistance * CountsPerDegree + e4);

        // Set target for rotation amount
        leftFront.setTargetPosition(LeftDrvMtrFntCnt_C);
        rightFront.setTargetPosition(RightDrvMtrFntCnt_C);
        leftRear.setTargetPosition(LeftDrvMtrCnt_C);
        rightRear.setTargetPosition(RightDrvMtrCnt_C);

        // Set motor modes
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power level for motors
        leftFront.setPower(0.37);
        rightFront.setPower(0.37);
        leftRear.setPower(0.37);
        rightRear.setPower(0.37);

        // runtime.reset();
        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
        }

        // Turn motors off
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

        // Return motors to mode for TeleOp operation
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void DriveBackwards() {
        // A. Calc encoder counts per inch of robot travel
        // Circumference of drive wheel = pi*dia
        // Which gives us travel per revolution of wheel
        InchesPerRev = 3.1416 * 4;
        // Calculate encoder counts per inch of travel
        // CpI = counts per motor rev / In per wheel rev
        // Valid for case where wheel is attached to motor hub
        CountsPerInch = 1440 / InchesPerRev;

        // Read current encoder position
        e1 = leftFront.getCurrentPosition();
        e2 = rightFront.getCurrentPosition();
        e3 = leftRear.getCurrentPosition();
        e4 = rightRear.getCurrentPosition();

        // Calculate position for rotation
        LeftDrvMtrFntCnt_C = (int) (TargetDistance * CountsPerInch + e1);
        RightDrvMtrFntCnt_C = (int) (-TargetDistance * CountsPerInch + e2);
        LeftDrvMtrCnt_C = (int) (-TargetDistance * CountsPerInch + e3);
        RightDrvMtrCnt_C = (int) (TargetDistance * CountsPerInch + e4);

        // Set target for rotation amount
        leftFront.setTargetPosition(LeftDrvMtrFntCnt_C);
        rightFront.setTargetPosition(RightDrvMtrFntCnt_C);
        leftRear.setTargetPosition(LeftDrvMtrCnt_C);
        rightRear.setTargetPosition(RightDrvMtrCnt_C);

        // Set motor modes
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power level for motors
        leftFront.setPower(0.9);
        rightFront.setPower(0.9);
        leftRear.setPower(0.9);
        rightRear.setPower(0.9);

        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
        }

        // Turn motors off
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

        // Return motors to mode for TeleOp operation
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision function-related IMU code  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVv */
    // IMU Methods
    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }


    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    /* VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV  Vision function-related IMU code END  VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVv */




}


