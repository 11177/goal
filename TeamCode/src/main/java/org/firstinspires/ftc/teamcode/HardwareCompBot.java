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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareCompBot {

    /* Public OpMode members. */
    DcMotor left1 = null;
    DcMotor right1 = null;
    DcMotor left2 = null;
    DcMotor right2 = null;
    Servo light = null;

    /* Constructor */
    HardwareCompBot() {

    }


    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        /* local OpMode members. */

        // Define and Initialize Motors
        left1 = ahwMap.get(DcMotor.class, "left1");
        right1 = ahwMap.get(DcMotor.class, "right1");
        left1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        right1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        left2 = ahwMap.get(DcMotor.class, "left2");
        right2 = ahwMap.get(DcMotor.class, "right2");
        left2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        right2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

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

        // Define and initialize ALL installed servos.

        light = ahwMap.get(Servo.class, "light");
        light.setPosition(.6395);
    }
}

