package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;

//@Disabled
@Autonomous(name = "disk", group = "disk")
public class AutoDraft extends LinearOpMode {

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;
    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
    private static float[] point1 = {4f / 8f + offsetX, 3.8f / 8f + offsetY};
    private static float[] point2 = {4f / 8f + offsetX, 3.9f / 8f + offsetY};
    private static float[] point3 = {4f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] point4 = {4f / 8f + offsetX, 4.1f / 8f + offsetY};
    private static float[] point5 = {4f / 8f + offsetX, 4.2f / 8f + offsetY};
    private static float[] point6 = {4f / 8f + offsetX, 4.3f / 8f + offsetY};
    private static float[] point7 = {4f / 8f + offsetX, 4.4f / 8f + offsetY};
    private static float[] point8 = {4f / 8f + offsetX, 4.5f / 8f + offsetY};
    private static int[] vals = {-1, -1, -1, -1, -1, -1, -1, -1};
    private final int rows = 640;
    private static int totals = 0;
    //moves all rectangles right or left by amount. units are in ratio to monitor
    private final int cols = 480;
    OpenCvCamera phoneCam;
    private ElapsedTime runtime = new ElapsedTime();
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
    private static final double P_DRIVE_COEFF = 0.05;
    private static int StartDistance = 0;
    private static int StartAngle = 0;
    private static int StartBack = 0;
    private static int ReturnDistance = 0;
    private static int ReturnAngle = 0;
    private static int drop2 = 0;
    private static int mid = 0;
    private static int backup2 = 0;

    HardwareCompBot robot = new HardwareCompBot();   // Use a Pushbot's hardware
    BNO055IMU imu;
    Orientation angles;



    public void runOpMode() throws InterruptedException {
        //if pos=A/none, go 74 inches if pos=6/1, go 97 If pos=c/4, go 121
        // turn, go forward 9 inches, and drop
        //turn back to normal, if pos=a/non go 0 if pos=b/1 go 24, if pos=c/4 go 48
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        runtime.reset();
        while (!isStarted()) {
            for (int i = 0; i <= 7; i++) {
                telemetry.addData("Values", vals[i]);
            }
            for (int i = 0; i <= 7; i++) {
                totals = totals + vals[i];
            }
            totals = totals / 255;
            telemetry.addData("total", totals);

            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);

            if (totals >= 4) {
                StartAngle = 30;
                StartDistance = 110;
                StartBack = -40;
                ReturnAngle = 28;
                ReturnDistance = -78;
                drop2 = 115;
                mid = 0;
                backup2 = -35;
            } else if (totals <= 0) {
                StartAngle = 55;
                StartDistance= 85;
                StartBack = -5;
               ReturnAngle = 45;
               ReturnDistance = -60;
               drop2 = 65;
               mid = -2;
               backup2 = -31;
            } else {
                StartAngle = 17;
                StartDistance = 75;
                StartBack = -20;
                ReturnAngle = 22;
                ReturnDistance = -50;
                drop2 = 90;
                mid = -8;
                backup2 = -40;
            }
            totals = 0;
        }



        gyroDrive(DRIVE_SPEED,6,0,30,0);
        gyroDrive(DRIVE_SPEED,36,-45,30,0);
        gyroDrive(DRIVE_SPEED,StartDistance,StartAngle,30,0);

        //robot.arm.setTargetPosition(-450);
        //robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /*robot.arm.setPower(-.1);
        //robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gyroHold(DRIVE_SPEED,0,2);
        robot.claw.setPosition(0);
        robot.arm.setPower(.05);
        gyroHold(DRIVE_SPEED,0,1);
        gyroDrive(DRIVE_SPEED,-10,0,5,0);
        gyroTurn(TURN_SPEED,90);
        gyroDrive(DRIVE_SPEED,-20,90,5,0);

         */
        gyroTurn(TURN_SPEED,ReturnAngle);
        robot.LWheel.setPower(-1);
        robot.RWheel.setPower(-1);
        gyroDrive(DRIVE_SPEED, ReturnDistance,ReturnAngle, 30,0);
        gyroHold(TURN_SPEED,3,1);
        robot.RTrack.setPosition(1);
        robot.LTrack.setPosition(0);
        gyroHold(TURN_SPEED,0,2);
        robot.LWheel.setPower(0);
        robot.RWheel.setPower(0);
        robot.RTrack.setPosition(.5);
        robot.LTrack.setPosition(.5);
        gyroTurn(TURN_SPEED,ReturnAngle);
        gyroDrive(DRIVE_SPEED, backup2, ReturnAngle, 30,0);

        gyroTurn(TURN_SPEED,90);
        gyroDrive(DRIVE_SPEED,52,80,5,0);
        gyroTurn(TURN_SPEED,65);
        gyroDrive(DRIVE_SPEED,drop2,2 + mid,10,0);
        robot.RDrop.setPosition(.23);
        robot.LDrop.setPosition(.23);
        gyroDrive(DRIVE_SPEED,StartBack,0,5,0);
        robot.arm.setPower(-.2);
        gyroHold(TURN_SPEED,0,1.7);
        robot.arm.setPower(0);
    }

    //detection pipeline
    class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        private opencvDetector.StageSwitchingPipeline.Stage stageToRenderToViewport = opencvDetector.StageSwitchingPipeline.Stage.detection;
        private opencvDetector.StageSwitchingPipeline.Stage[] stages = opencvDetector.StageSwitchingPipeline.Stage.values();

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
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 100, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object

            //get values from frame
            double[] pix1 = thresholdMat.get((int) (input.rows() * point1[1]), (int) (input.cols() * point1[0]));//gets value at circle
            vals[0] = (int) pix1[0];
            double[] pix2 = thresholdMat.get((int) (input.rows() * point2[1]), (int) (input.cols() * point2[0]));//gets value at circle
            vals[1] = (int) pix2[0];
            double[] pix3 = thresholdMat.get((int) (input.rows() * point3[1]), (int) (input.cols() * point3[0]));//gets value at circle
            vals[2] = (int) pix3[0];
            double[] pix4 = thresholdMat.get((int) (input.rows() * point4[1]), (int) (input.cols() * point4[0]));//gets value at circle
            vals[3] = (int) pix4[0];
            double[] pix5 = thresholdMat.get((int) (input.rows() * point5[1]), (int) (input.cols() * point5[0]));//gets value at circle
            vals[4] = (int) pix5[0];
            double[] pix6 = thresholdMat.get((int) (input.rows() * point6[1]), (int) (input.cols() * point6[0]));//gets value at circle
            vals[5] = (int) pix6[0];
            double[] pix7 = thresholdMat.get((int) (input.rows() * point7[1]), (int) (input.cols() * point7[0]));//gets value at circle
            vals[6] = (int) pix7[0];
            double[] pix8 = thresholdMat.get((int) (input.rows() * point8[1]), (int) (input.cols() * point8[0]));//gets value at circle
            vals[7] = (int) pix8[0];
            //create three points
            Point Point1 = new Point((int) (input.cols() * point1[0]), (int) (input.rows() * point1[1]));
            Point Point2 = new Point((int) (input.cols() * point2[0]), (int) (input.rows() * point2[1]));
            Point Point3 = new Point((int) (input.cols() * point3[0]), (int) (input.rows() * point3[1]));
            Point Point4 = new Point((int) (input.cols() * point4[0]), (int) (input.rows() * point4[1]));
            Point Point5 = new Point((int) (input.cols() * point5[0]), (int) (input.rows() * point5[1]));
            Point Point6 = new Point((int) (input.cols() * point6[0]), (int) (input.rows() * point6[1]));
            Point Point7 = new Point((int) (input.cols() * point7[0]), (int) (input.rows() * point7[1]));
            Point Point8 = new Point((int) (input.cols() * point8[0]), (int) (input.rows() * point8[1]));

            //draw circles on those points
            Imgproc.circle(all, Point1, 3, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, Point2, 3, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, Point3, 3, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, Point4, 3, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, Point5, 3, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, Point6, 3, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, Point7, 3, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, Point8, 3, new Scalar(255, 0, 0), 1);//draws circle

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
    }


    private void gyroDrive(double driveSpeed, //never mess up the speed
                           double distance,
                           double angle,
                           double timed,
                           double arm ) {

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
        double armpower;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget1 = robot.DriveLeft1.getCurrentPosition() + moveCounts;
            newRightTarget1 = robot.DriveRight1.getCurrentPosition() + moveCounts;
            newLeftTarget2 = robot.DriveLeft2.getCurrentPosition() + moveCounts;
            newRightTarget2 = robot.DriveRight2.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.DriveLeft1.setTargetPosition(newLeftTarget1);
            robot.DriveRight1.setTargetPosition(newRightTarget1);
            robot.DriveLeft2.setTargetPosition(newLeftTarget2);
            robot.DriveRight2.setTargetPosition(newRightTarget2);
            robot.DriveLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.DriveLeft1.setPower(speed);
            robot.DriveRight1.setPower(speed);
            robot.DriveLeft2.setPower(speed);
            robot.DriveRight2.setPower(speed);

            //new timer to time out drive step
            ElapsedTime holdTimer = new ElapsedTime();
            // keep looping while we have time remaining.
            holdTimer.reset();

            // keep looping while we are still active, not past timer, and all motors are running.  This will stop if any are false
            while (opModeIsActive() && holdTimer.time() < timed &&
                    (robot.DriveLeft1.isBusy() && robot.DriveRight1.isBusy() && robot.DriveLeft2.isBusy() && robot.DriveRight2.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);
                /*
                // Slow down to help with control and prevent skidding
                currentReading = robot.right1.getCurrentPosition(); //this part is a slowdown to help reduce mess-ups, especially when grabbing block.

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

                robot.DriveLeft1.setPower(leftSpeed);
                robot.DriveRight1.setPower(rightSpeed);
                robot.DriveLeft2.setPower(leftSpeed);
                robot.DriveRight2.setPower(rightSpeed);
                if (robot.arm.getCurrentPosition()>arm+5){
                    armpower=robot.arm.getPower();
                    robot.arm.setPower(armpower-.0005);
                } else if (robot.arm.getCurrentPosition()<arm-5){
                    armpower=robot.arm.getPower();
                    robot.arm.setPower(armpower+.0005);
                } else robot.arm.setPower(0);

            }

            // Stop all motion;
            robot.DriveLeft1.setPower(0);
            robot.DriveRight1.setPower(0);
            robot.DriveLeft2.setPower(0);
            robot.DriveRight2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.DriveLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
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
        robot.DriveLeft1.setPower(0);
        robot.DriveRight1.setPower(0);
        robot.DriveLeft2.setPower(0);
        robot.DriveRight2.setPower(0);
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
        robot.DriveLeft1.setPower(leftSpeed);
        robot.DriveRight1.setPower(rightSpeed);
        robot.DriveLeft2.setPower(leftSpeed);
        robot.DriveRight2.setPower(rightSpeed);
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