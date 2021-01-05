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


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Drive_Robot", group = "Pushbot")
//@Disabled
public class Drive_Mode extends OpMode {

    /* Declare OpMode members. */
    HardwareCompBot robot = new HardwareCompBot(); // use the class created to define a Pushbot's hardware
    private static ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);
        robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.track.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.track.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //view.runOpMode();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.addData("arm position", robot.arm.getCurrentPosition());
        telemetry.addData("track position", robot.track.getCurrentPosition());
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
        double left;
        double right;
        double armPower;
        // Changes the led color for end game
        if (runtime.seconds() >= 110) {
            robot.light.setPosition(.2775);
        } else if (runtime.seconds() >= 85) {
            robot.light.setPosition(.3375);
        } else robot.light.setPosition(.6545);

        // wheel control
        if (gamepad2.dpad_up) {
            robot.track.setPower(-1);
        } else if (gamepad2.dpad_down) {
            robot.track.setPower(1);
        } else {
            robot.track.setPower(0);
        }

        // this will open and close the claw
        if (gamepad2.left_bumper) {
            robot.leftClaw.setPosition(-1);
            robot.rightClaw.setPosition(1);
        } else if (gamepad2.right_bumper) {
            robot.leftClaw.setPosition(1);
            robot.rightClaw.setPosition(-1);
        }
        //this is for the cap stone controls
        if (gamepad1.x) {
            robot.push.setPosition(.75);
        }
        if (gamepad1.b) {
            robot.push.setPosition(.25);
        }
        if (gamepad1.y) {
            robot.cube.setPosition(1);
            robot.drag.setPosition(0);
        }
        if (gamepad1.a) {
            robot.cube.setPosition(-.5);
            robot.drag.setPosition(1);
        }

        //  this will cut the power to the wheel by 50 and 75 percent if needed for control
        if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0) {
                left = -gamepad1.left_stick_y / 4;
                right = -gamepad1.right_stick_y / 4;
            } else {
                left = -gamepad1.left_stick_y / 2;
                right = -gamepad1.right_stick_y / 2;
            }
        } else {
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
        }

        //  controls the arm.  gives constant power to hold arm in air.  Cuts down power percent to prevent damage
        armPower = -gamepad2.left_stick_y;
        if (gamepad2.left_stick_button) {
            armPower = -.15;
        } else if (armPower < 0) {
            armPower = ((armPower / 10) + .05);
        } else if (armPower > 0) {
            armPower = armPower / 2;
        } else {
            armPower = .1;
        }
        robot.arm.setPower(armPower);

        //  THis is for side ways movement changes the power to the wheels.  Power level is still the same as above
        if (gamepad1.left_bumper) {
            robot.left1.setPower(-left);
            robot.right1.setPower(right);
            robot.left2.setPower(left);
            robot.right2.setPower(-right);
        } else if (gamepad1.right_bumper) {
            robot.left1.setPower(left);
            robot.right1.setPower(-right);
            robot.left2.setPower(-left);
            robot.right2.setPower(right);
        } else {
            robot.left1.setPower(left);
            robot.right1.setPower(right);
            robot.left2.setPower(left);
            robot.right2.setPower(right);
        }
        if (gamepad1.left_stick_button) {
            robot.arm.setPower(-.2);
        }
        if (gamepad1.right_stick_button) {
            robot.arm.setPower(.7);
        }
        telemetry.addData("left", "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("armPower", "%.2f", armPower);
        telemetry.addData("arm position", robot.arm.getCurrentPosition());
        telemetry.addData("Track", robot.track.getCurrentPosition());
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
