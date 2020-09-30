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

package org.firstinspires.ftc.promoBot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "testdrive", group = "test robot")
@Disabled
public class test_robo extends OpMode {


    /* Declare OpMode members. */
    private HardwareTestBot testrobot = new HardwareTestBot(); // use the class created to define a Pushbot's hardware


    /* local OpMode members. */
    private HardwareMap hwMap = null;

    /* Constructor */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         * Hi
         */
        testrobot.init(hardwareMap);
        //view.runOpMode();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        if (gamepad1.a){
            testrobot.drag.setPosition(1);
        }
        if (gamepad1.x){
            testrobot.drag.setPosition(0);
        }


        if (gamepad1.left_bumper) {
            testrobot.left1.setPower(-left);
            testrobot.right1.setPower(right);
            testrobot.left2.setPower(left);
            testrobot.right2.setPower(-right);
        } else if (gamepad1.right_bumper) {
            testrobot.left1.setPower(left);
            testrobot.right1.setPower(-right);
            testrobot.left2.setPower(-left);
            testrobot.right2.setPower(right);
        } else {
            testrobot.left1.setPower(left);
            testrobot.right1.setPower(right);
            testrobot.left2.setPower(left);
            testrobot.right2.setPower(right);
        }
    }



/*
     * Code to run ONCE after the driver hits STOP
     * never forget
     */
    @Override
    public void stop() {
    }

    static class HardwareTestBot {
        /* Public OpMode members. */
        public DcMotor left1 = null;
        public DcMotor right1 = null;
        public DcMotor left2 = null;
        public DcMotor right2 = null;
        public Servo drag = null;

        /* local OpMode members. */
        private HardwareMap hwMap = null;

        /* Constructor */
        public HardwareTestBot() {

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors
            left1 = hwMap.get(DcMotor.class, "left1");
            right1 = hwMap.get(DcMotor.class, "right1");
            left1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            right1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            left2 = hwMap.get(DcMotor.class, "left2");
            right2 = hwMap.get(DcMotor.class, "right2");
            left2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            right2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            drag = ahwMap.get(Servo.class, "drag");

            // Set all motors to zero power
            left1.setPower(0);
            right1.setPower(0);
            left2.setPower(0);
            right2.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }
}


