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
    DcMotor DriveLeft1 = null;
    DcMotor DriveRight1 = null;
    DcMotor DriveLeft2 = null;
    DcMotor DriveRight2 = null;
    Servo light = null;
    DcMotor arm = null;
    Servo claw = null;

    /*
    DcMotor LaunchL = null;

    DcMotor LaunchR = null;
    DcMotor SpinIn = null;
    DcMotor Ramp = null;
    Servo DrawBridge = null;
    Servo Stopper = null;
*/

    /* Constructor */
    HardwareCompBot() {

    }


    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        /* local OpMode members. */

        // Define and Initialize Motors
        DriveLeft1 = ahwMap.get(DcMotor.class, "DriveLeft1");
        DriveRight1 = ahwMap.get(DcMotor.class, "DriveRight1");
        DriveLeft1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        DriveRight1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        DriveLeft2 = ahwMap.get(DcMotor.class, "DriveLeft2");
        DriveRight2 = ahwMap.get(DcMotor.class, "DriveRight2");
        DriveLeft2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        DriveRight2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        DriveLeft1.setPower(0);
        DriveRight1.setPower(0);
        DriveLeft2.setPower(0);
        DriveRight2.setPower(0);
        arm.setPower(0);
  /*
        LaunchL.setPower(0);
        LaunchR.setPower(0);
        SpinIn.setPower(0);
        Ramp.setPower(0);
*/

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        DriveLeft1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveRight1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveLeft2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveRight2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        LaunchL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LaunchR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SpinIn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Ramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/
        // Define and initialize ALL installed servos.


     /*   DrawBridge = ahwMap.get(Servo.class, "DrawBridge");
        DrawBridge.setPosition(0);
        Stopper = ahwMap.get(Servo.class, "Stopper");
        Stopper.setPosition(0);

      */
       /* arm = ahwMap.get(Servo.class, "arm");
        arm.setPosition(.1);


        */
        claw = ahwMap.get(Servo.class, "claw");
        claw.setPosition(0);

        light = ahwMap.get(Servo.class, "light");
        light.setPosition(.6975);

    }
}

