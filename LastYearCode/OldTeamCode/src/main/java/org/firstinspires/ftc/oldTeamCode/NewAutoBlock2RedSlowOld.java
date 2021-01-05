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
package org.firstinspires.ftc.oldTeamCode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

/*
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * <p>
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.
 * <p>
 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 * <p>
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


@Autonomous(name = "Get Red Stones", group = "Pushbot")
@Disabled
public class NewAutoBlock2RedSlowOld extends LinearOpMode {

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
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
     */
    private static final String VUFORIA_KEY =
            "AUE/X+f/////AAABmatHKAGBoEP+tPf7XqQ6igxpkoMptkpSJnnXVIODw8Pb88W//Pei+bCs2mZdsZmg+QADtsTlZrPh75PlqN2MZ296CxQuh5LV++pv9rcQ7VPuZYa1fwh95yavxCrexKda84s+ianByrTqWjg8/GI35fKzD/0qayI7KCpXUawB4pPM+8f4gtVdohmMkOylsHZxigxAwXfNAnyEjIU+hOszJnYtLPO+9f88pGbQhzn3lCS3G1HpYEpdBHm724v5s4c1EIGN08fs9DSwU2gsOQk6erzUKb7STFwcwsL4sLwxnJBZmBZK287fYGGLQ3gnIP9kixXnLAp4iux+J1JkOICKwf/S7xujNBOCRLnE+8c5rFnL";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    //private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;
    // Constants for the center support targets
    /*
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;
    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
     */
    private static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: neverest 40 Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    private static final double DRIVE_SPEED = .9;     // Nominal speed for better accuracy.
    private static final double TURN_SPEED = 0.8;     // Nominal half speed for better accuracy.
    private static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    private static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF = 0.05;     // Larger is more responsive, but also less stable
    /* Declare OpMode members. */
    HardwareCompBot robot = new HardwareCompBot();   // Use a Pushbot's hardware
    BNO055IMU imu;
    Orientation angles;
    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    private int RoB = 1;
    private String block = null;
    private double turnAngle;
    private double toBlock;


    @Override
    public void runOpMode() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        /*
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

        */


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<>(targetsSkyStone);

        /*
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

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        /*
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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


         */
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
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        } else{
            //phone is head to right so need to rotate 180 degrees
            phoneXRotate = 180;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 8.5f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 2.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 2.0f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /*  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.track.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.track.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();


        targetsSkyStone.activate();


        while (!isStarted()) {


        }


        robot.arm.setPower(.1);
        robot.track.setTargetPosition(-8000);
        robot.track.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.track.setPower(-1);
        gyroDrive(DRIVE_SPEED, 16, 0, 30);
        robot.arm.setPower(0);
        while (robot.track.isBusy() && opModeIsActive()) {
            targetVisible = false;
            while (robot.track.isBusy() && opModeIsActive() && block == null) {
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;
                        if (trackable.getName() == "Stone Target") {
                            block = "yay";
                        }

                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }

                }


                if (targetVisible) {
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                    if ((translation.get(1)) > 0 && block == "yay") {
                        block = "center";
                        toBlock = 8;
                        turnAngle = -10;
                        telemetry.addData("Visible Target", "center block");
                    } else if ((translation.get(1)) < 0 && block == "yay") {
                        block = "left";
                        toBlock = 10;
                        turnAngle = 10;
                        telemetry.addData("Visible Target", "left block");
                    }
                    telemetry.addData("Y reading", "{Y} = %.1f", translation.get(1));
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                    telemetry.update();
                }

            }
            robot.arm.setPower(-.2);
            if(block == null){
                block = "right";
                toBlock = 11;
                turnAngle = -25;
                telemetry.addData("Visible Target", "right block");
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                telemetry.update();
            }
        }
        targetsSkyStone.deactivate();
        robot.track.setPower(0);
        robot.track.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gyroTurn(TURN_SPEED, turnAngle);
        gyroHold(TURN_SPEED, turnAngle, 0.2);
        robot.arm.setPower(0);
        gyroDrive(DRIVE_SPEED, toBlock, turnAngle, 30);
        robot.leftClaw.setPosition(1);
        robot.rightClaw.setPosition(-1);
        gyroHold(TURN_SPEED, turnAngle, .7);
        gyroDrive(DRIVE_SPEED, -toBlock, turnAngle,30);
        gyroTurn(TURN_SPEED, -80 * RoB);
        gyroDrive(DRIVE_SPEED, 55, -80 * RoB,30);
        robot.leftClaw.setPosition(-1);
        robot.rightClaw.setPosition(1);
        gyroDrive(DRIVE_SPEED, -76, -80 * RoB,30);
        robot.leftClaw.setPosition(0);
        robot.rightClaw.setPosition(0);
        gyroTurn(TURN_SPEED, turnAngle + 10);
        gyroHold(TURN_SPEED, turnAngle + 10, .2);
        gyroDrive(DRIVE_SPEED, toBlock + 2, turnAngle + 10,30);
        robot.leftClaw.setPosition(1);
        robot.rightClaw.setPosition(-1);
        gyroHold(TURN_SPEED, turnAngle + 10, .7);
        gyroDrive(DRIVE_SPEED, -toBlock - 2, turnAngle + 10,30);
        gyroTurn(TURN_SPEED, -80 * RoB);
        gyroDrive(DRIVE_SPEED, 75, -80 * RoB,30);
        robot.leftClaw.setPosition(-1);
        robot.rightClaw.setPosition(1);
        gyroDrive(DRIVE_SPEED, -18, -80 * RoB,30);


        telemetry.addData("Path", "Complete");
        telemetry.update();


        while (!isStopRequested()) {
            robot.arm.setPower(0);
            robot.track.setPower(0);
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);

        }

        // Disable Tracking when we are done;

        robot.arm.setPower(0);
        robot.track.setPower(0);
        robot.left1.setPower(0);
        robot.left2.setPower(0);
        robot.right1.setPower(0);
        robot.right2.setPower(0);
    }

    /*
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     */
    private void gyroDrive(double driveSpeed,
                           double distance,
                           double angle,
                           double timed) {

        int newLeftTarget1;
        int newRightTarget1;
        int newLeftTarget2;
        int newRightTarget2;
        int moveCounts;
        double speed = driveSpeed;
        double currentReading;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget1 = robot.left1.getCurrentPosition() + moveCounts;
            newRightTarget1 = robot.right1.getCurrentPosition() + moveCounts;
            newLeftTarget2 = robot.left2.getCurrentPosition() + moveCounts;
            newRightTarget2 = robot.right2.getCurrentPosition() + moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            robot.left1.setTargetPosition(newLeftTarget1);
            robot.right1.setTargetPosition(newRightTarget1);
            robot.left2.setTargetPosition(newLeftTarget2);
            robot.right2.setTargetPosition(newRightTarget2);

            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.left1.setPower(speed);
            robot.right1.setPower(speed);
            robot.left2.setPower(speed);
            robot.right2.setPower(speed);

            //new timer to time out drive step
            ElapsedTime holdTimer = new ElapsedTime();
            // keep looping while we have time remaining.
            holdTimer.reset();

            // keep looping while we are still active, and BOTH motors are running.

            while (opModeIsActive() && holdTimer.time() < timed &&
                    (robot.left1.isBusy() && robot.right1.isBusy() && robot.left2.isBusy() && robot.right2.isBusy())) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                //new code For speed reduction
                currentReading = robot.right1.getCurrentPosition();
                if (distance>0){
                    if (currentReading >(newRightTarget1-(6*COUNTS_PER_INCH))){
                        speed = driveSpeed/2;
                        if (currentReading >(newRightTarget1-(3*COUNTS_PER_INCH))){
                            speed = driveSpeed/4;
                        }
                    }
                } else {
                    if (currentReading <(newRightTarget1+(6*COUNTS_PER_INCH))){
                        speed = driveSpeed/2;
                        if (currentReading <(newRightTarget1+(3*COUNTS_PER_INCH))){
                            speed = driveSpeed/4;
                        }
                    }
                }

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.left1.setPower(leftSpeed);
                robot.right1.setPower(rightSpeed);
                robot.left2.setPower(leftSpeed);
                robot.right2.setPower(rightSpeed);

            }

            // Stop all motion;
            robot.left1.setPower(0);
            robot.right1.setPower(0);
            robot.left2.setPower(0);
            robot.right2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    /*
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        //ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /*
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            //telemetry.update();
        }

        // Stop all motion;
        robot.left1.setPower(0);
        robot.right1.setPower(0);
        robot.left2.setPower(0);
        robot.right2.setPower(0);
    }

    /*
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    private boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.left1.setPower(leftSpeed);
        robot.right1.setPower(rightSpeed);
        robot.left2.setPower(leftSpeed);
        robot.right2.setPower(rightSpeed);
        /*
        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);


         */
        return onTarget;
    }

    /*
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /*
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


}
