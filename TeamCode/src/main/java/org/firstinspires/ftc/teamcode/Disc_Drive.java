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


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Drive_Robot", group = "Pushbot")
//@Disabled
public class Disc_Drive extends OpMode {

    /* Declare OpMode members. */
    HardwareCompBot robot = new HardwareCompBot(); // use the class created to define a Pushbot's hardware
    private static ElapsedTime runtime = new ElapsedTime();
    double left;
    double right;
    double arm = 0;
    double armpower;
    double current;
    double wheel;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //robot.track.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //view.runOpMode();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {

        // Changes the led color for end game
     /*   if (runtime.seconds() >= 110) {
            robot.light.setPosition(.2775);
        } else if (runtime.seconds() >= 85) {
            robot.light.setPosition(.3375);
        } else robot.light.setPosition(.6545);
       */
        if (gamepad2.left_bumper) {
            robot.claw.setPosition(1);
        } else if (gamepad2.right_bumper) {
            robot.claw.setPosition(0);
        }
        //  this will cut the power to the wheel by 50 and 75 percent if needed for control
        if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0) {
                left = -gamepad1.left_stick_y / 4;
                right= -gamepad1.right_stick_y / 4;
            } else {
                left = -gamepad1.left_stick_y / 2;
                right = -gamepad1.right_stick_y / 2;
            }
        } else {
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
        }

        // wheel test right thumb controller 2
        wheel = -gamepad2.right_stick_y;
        robot.LWheel.setPower(wheel);
        robot.RWheel.setPower(-wheel);

        if (gamepad2.x){
            robot.RWheel.setPower(0);
            robot.LWheel.setPower(0);
        }

        if (gamepad2.left_stick_y!=0){
            robot.arm.setPower(-gamepad2.left_stick_y/6);
            arm = robot.arm.getCurrentPosition();
        }else if (robot.arm.getCurrentPosition()>arm+5){
            armpower=robot.arm.getPower();
            robot.arm.setPower(armpower-.0005);
        } else if (robot.arm.getCurrentPosition()<arm-5){
            armpower=robot.arm.getPower();
            robot.arm.setPower(armpower+.0005);
        } else robot.arm.setPower(0);

        armpower=robot.arm.getPower();
        if (gamepad1.left_bumper) {
            robot.DriveLeft1.setPower(-left);
            robot.DriveRight1.setPower(right);
            robot.DriveLeft2.setPower(left);
            robot.DriveRight2.setPower(-right);
        } else if (gamepad1.right_bumper) {
            robot.DriveLeft1.setPower(left);
            robot.DriveRight1.setPower(-right);
            robot.DriveLeft2.setPower(-left);
            robot.DriveRight2.setPower(right);
        } else {
            robot.DriveLeft1.setPower(left);
            robot.DriveRight1.setPower(right);
            robot.DriveLeft2.setPower(left);
            robot.DriveRight2.setPower(right);
        }
        current=robot.arm.getCurrentPosition();
        telemetry.addData("left", "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("ArmPower", "%.2f", armpower);
        telemetry.addData("arm", "%.2f", arm);
        telemetry.addData("ArmCurrent", "%.2f", current);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
