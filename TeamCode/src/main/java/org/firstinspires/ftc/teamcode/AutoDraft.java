package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Disabled
@Autonomous(name = "disk", group = "disk")
public class AutoDraft extends LinearOpMode {

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;
    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
    private static float[] point1 = {4f / 8f + offsetX, 2.6f / 8f + offsetY};
    private static float[] point2 = {4f / 8f + offsetX, 2.7f / 8f + offsetY};
    private static float[] point3 = {4f / 8f + offsetX, 2.8f / 8f + offsetY};
    private static float[] point4 = {4f / 8f + offsetX, 2.9f / 8f + offsetY};
    private static float[] point5 = {4f / 8f + offsetX, 3f / 8f + offsetY};
    private static float[] point6 = {4f / 8f + offsetX, 3.1f / 8f + offsetY};
    private static float[] point7 = {4f / 8f + offsetX, 3.2f / 8f + offsetY};
    private static float[] point8 = {4f / 8f + offsetX, 3.3f / 8f + offsetY};
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
    private static int DistanceStart = 0;
    private static int TurnStart = 0;
    private static int backstart =0;

    HardwareCompBot robot = new HardwareCompBot();   // Use a Pushbot's hardware

    public void runOpMode() throws InterruptedException {
        //if pos=A/none, go 74 inches if pos=6/1, go 97 If pos=c/4, go 121
        // turn, go forward 9 inches, and drop
        //tun back to normal, if pos=a/non go 0 if pos=b/1 go 24, if pos=c/4 go 48
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_RIGHT);//display on RC
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
                TurnStart = -90;
                DistanceStart = 121;
                backstart =48;
            } else if (totals <= 1) {
                TurnStart = -90;
                DistanceStart = 74;
                backstart = 0;
            } else {
                TurnStart = 90;
                DistanceStart = 97;
                backstart = 24;
            }
            totals = 0;
        }
        while (opModeIsActive()) {
        gyroDrive(DRIVE_SPEED, DistanceStart,0, 30);
        gyroTurn(TurnStart, TURN_SPEED);
        gyroDrive(DRIVE_SPEED, 6, TurnStart, 30);
        gyroDrive(DRIVE_SPEED, -6, TurnStart, 30);
        gyroTurn(TURN_SPEED, -TurnStart);
        gyroDrive(DRIVE_SPEED, backstart,0, 30);
        gyroHold(DRIVE_SPEED, 0, 30);
        }
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
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 75, 255, Imgproc.THRESH_BINARY_INV);

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

    }public void gyroTurn(double speed, double angle) {

        //ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive()) {
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

    private void onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

    }

}