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

@Autonomous(name = "Tray_line", group = "Pushbot")
//@Disabled
public class tray_line extends LinearOpMode {
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
    private int RoB = 1;
    private boolean done;


    @Override
    public void runOpMode() {

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


        // make sure the gyro is calibrated before continuing
        while (!isStarted()) {

        }
        //Run Program
        robot.track.setTargetPosition(-9250);
        robot.track.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.track.setPower(-1);
        robot.arm.setPower(0);
        done = false;
        //run track out while moving to tray
        while (robot.track.isBusy() && opModeIsActive()) {
            if (!done) {
                gyroDrive(DRIVE_SPEED / 2, -18, 0, 30);
                done = true;

            }
        }
        robot.track.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.track.setPower(0);
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
     * I am ertanal
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
