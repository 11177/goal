package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


/**
 * Created by maryjaneb  on 11/13/2016.
 * <p>
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 * <p>
 * monitor: 640 x 480
 * YES
 */
@Autonomous(name = "opencvSkystoneDetector", group = "Sky autonomous")
//@Disabled//comment out this line before using
public class opencvDetector extends LinearOpMode {
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valTall = -1;
    private static int valMid = -1;
    private static int valNone = -1;
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
    private final int rows = 640;
    //moves all rectangles right or left by amount. units are in ratio to monitor
    private final int cols = 480;
    OpenCvCamera phoneCam;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_RIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valTall + "   " + valMid + "   " + valNone);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);
        }
    }

    //detection pipeline
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
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object

            //get values from frame
            double[] pix1 = thresholdMat.get((int) (input.rows() * point1[1]), (int) (input.cols() * point1[0]));//gets value at circle
            valMid = (int) pix1[0];
            double[] pix2 = thresholdMat.get((int) (input.rows() * point2[1]), (int) (input.cols() * point2[0]));//gets value at circle
            valTall = (int) pix2[0];
            double[] pix3 = thresholdMat.get((int) (input.rows() * point3[1]), (int) (input.cols() * point3[0]));//gets value at circle
            valTall = (int) pix3[0];
            double[] pix4 = thresholdMat.get((int) (input.rows() * point4[1]), (int) (input.cols() * point4[0]));//gets value at circle
            valTall = (int) pix4[0];
            double[] pix5 = thresholdMat.get((int) (input.rows() * point5[1]), (int) (input.cols() * point5[0]));//gets value at circle
            valTall = (int) pix5[0];
            double[] pix6 = thresholdMat.get((int) (input.rows() * point6[1]), (int) (input.cols() * point6[0]));//gets value at circle
            valTall = (int) pix6[0];
            double[] pix7 = thresholdMat.get((int) (input.rows() * point7[1]), (int) (input.cols() * point7[0]));//gets value at circle
            valTall = (int) pix7[0];
            double[] pix8 = thresholdMat.get((int) (input.rows() * point8[1]), (int) (input.cols() * point8[0]));//gets value at circle
            valTall = (int) pix8[0];
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

                case RAW_IMAGE: {return input;}

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
}