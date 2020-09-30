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

package org.firstinspires.ftc.teamCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "blue_Stones", group = "Pushbot")
//@Disabled
public class Blue_Stones extends LinearOpMode {
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
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;
    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;
    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
    private static float[] midPos = {3.75f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {5.5f / 8f + offsetX, 4f / 8f + offsetY};
    private final int rows = 640;
    //moves all rectangles right or left by amount. units are in ratio to monitor
    private final int cols = 480;
    OpenCvCamera phoneCam;
    /* Declare OpMode members. */
    HardwareCompBot robot = new HardwareCompBot();   // Use a Pushbot's hardware
    BNO055IMU imu;
    Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();

    // Class Members
    private int RoB = -1;
    private boolean done;
    private String block = null;
    private double turnAngle; //the angle for the 2nd block
    private double toBlock; //the distance to the 2nd block, and backup from the 1st.
    private double startturn; //first angle from the start to grab the 1st block.
    private double startdistance; //the distance to the 1st block from start

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_RIGHT);//display on RC

        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro. Numbers are off without this, leads to disaster if not in.
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
        telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
        telemetry.update();
        sleep(100);

        // make sure the gyro is calibrated before continuing
        while (!isStarted()) {
            // Send telemetry message to signify robot waiting;
            telemetry.addData(">", "Robot Ready.");    //
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            telemetry.update();
            sleep(100);
            //The if is for the skystone
            if (valLeft == 0) {   // Stone on left
                startdistance = 23;
                startturn = 12;
                toBlock = 4;
                turnAngle = 0;
                robot.light.setPosition(.5545); //each one of these is for color of the LED lights
            } else if (valMid == 0) {  // Stone in center
                startdistance = 21;
                startturn = 0;
                toBlock = 4;
                turnAngle = -10;
                robot.light.setPosition(.4195);
            } else if (valRight == 0) {  // Stone on right
                startdistance = 23;
                startturn = -12;
                toBlock = 5;
                turnAngle = -20;
                robot.light.setPosition(.4995);
            } else { //goes for the right block
                startdistance = 23;
                startturn = -12;
                toBlock = 5;
                turnAngle = -20;
                robot.light.setPosition(.6295);
            }
        }
        robot.light.setPosition(.6395);
        //now we run
        runtime.reset();
        phoneCam.closeCameraDevice();//close camera
        robot.arm.setPower(0);
        robot.track.setTargetPosition(-22000);
        robot.track.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.track.setPower(-1);
        done = false;
        //run le track out while moving to block
        while (robot.track.isBusy() && opModeIsActive()) {
            if (!done) {
                gyroDrive(DRIVE_SPEED, startdistance, startturn, 10);
                done = true;
                gyroHold(TURN_SPEED, startturn, .75); //need this to give claw time to grab
                robot.arm.setPower(-.05);
            }
        }
        robot.track.setPower(0);
        robot.track.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setPower(0); //we keep the arm down entire time, no problems.
        //we are at the block now
        robot.leftClaw.setPosition(1);
        robot.rightClaw.setPosition(-1);
        gyroHold(TURN_SPEED, startturn, .75); //need this to give claw time to grab
        gyroDrive(DRIVE_SPEED, -toBlock, startturn, 30);
        gyroTurn(TURN_SPEED, -80 * RoB); //stay on our part of the line by being at an angle
        gyroHold(TURN_SPEED, -80 * RoB, 0.25);
        gyroDrive(DRIVE_SPEED, 55, -80 * RoB, 30);
        robot.leftClaw.setPosition(-1);
        robot.rightClaw.setPosition(1);
        //drop the block past the line
        gyroDrive(DRIVE_SPEED, -73, -80 * RoB, 30); //oh we are coming back for number 2!
        gyroTurn(TURN_SPEED, turnAngle);
        gyroHold(TURN_SPEED, turnAngle, .5);
        gyroDrive(DRIVE_SPEED, toBlock + 6, turnAngle, 30);
        gyroHold(TURN_SPEED, turnAngle, .25);
        robot.leftClaw.setPosition(1);
        robot.rightClaw.setPosition(-1);
        gyroHold(TURN_SPEED, turnAngle, .75);  //wait for claw again, crucial.
        gyroDrive(DRIVE_SPEED, -toBlock - 6, turnAngle, 30);
        gyroTurn(TURN_SPEED, -80 * RoB);
        gyroHold(TURN_SPEED, -80 * RoB, 0.25);
        gyroDrive(DRIVE_SPEED, 72, -80 * RoB, 30);
        robot.leftClaw.setPosition(-1);
        robot.rightClaw.setPosition(1); //dropping 2nd block next to 1st one.
        gyroDrive(DRIVE_SPEED, -24, -94 * RoB, 6);  //back to line at an angle.
        gyroTurn(TURN_SPEED, -90 * RoB);
        gyroDrive(DRIVE_SPEED, -2, -90 * RoB, 1); //park correctly


        telemetry.addData("Path", "Complete");
        telemetry.update();


        //kills motors after program is ran and waits out timer to end program
        while (!isStopRequested()) {
            robot.arm.setPower(0);
            robot.track.setPower(0);
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);

        }

        //stops everything when we press stop, need it to pass inspection.
        robot.arm.setPower(0);
        robot.track.setPower(0);
        robot.left1.setPower(0);
        robot.left2.setPower(0);
        robot.right1.setPower(0);
        robot.right2.setPower(0);
    }

    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            valMid = (int) pixMid[0];

            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];

            //create three points
            Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth / 2),
                            input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth / 2),
                            input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

    }

    /* All the driving Magic!!
     * GyroDrive to move robot
     * GyroTurn to turn robot
     * GyroHold to hold robot at angle and ensure pause
     */

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
     *
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


        // Ensure that teh opmode is still active
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
                /*
                currentReading = robot.right1.getCurrentPosition();
                if (distance > 0) {
                    if (currentReading > (newRightTarget1 - (6 * COUNTS_PER_INCH))) {
                        speed = driveSpeed * .75;
                        if (currentReading > (newRightTarget1 - (3 * COUNTS_PER_INCH))) {
                            speed = driveSpeed * .5;
                        }
                    }
                } else {
                    if (currentReading < (newRightTarget1 + (6 * COUNTS_PER_INCH))) {
                        speed = driveSpeed * .75;
                        if (currentReading < (newRightTarget1 + (3 * COUNTS_PER_INCH))) {
                            speed = driveSpeed * .5;
                        }
                    }
                }
                */
                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer; //left is reversed
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
     *
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


}
