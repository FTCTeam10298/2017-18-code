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

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for our robot.
 */

public class OurBot_Hardware
{
    /* Public OpMode members. */
    public DcMotor leftDrive      = null;
    public DcMotor rightDrive     = null;
    public DcMotor leftSlide      = null;
    public DcMotor rightSlide     = null;
    public DcMotor dunkClawArm = null;
    public DcMotor slideClaw      = null;
    public DcMotor jewelArm       = null;
    public Servo   dunkClawLeft1  = null;
    public Servo   dunkClawRight1 = null;
    public Servo   dunkClawLeft2  = null;
    public Servo   dunkClawRight2 = null;

    /* local OpMode members. */
    HardwareMap     hwMap         = null;

    /* Constructor */
    public OurBot_Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftSlide  = hwMap.get(DcMotor.class, "left_arm");
        rightSlide = hwMap.get(DcMotor.class, "right_arm");
        dunkClawArm = hwMap.get(DcMotor.class, "back_arm");
        slideClaw  = hwMap.get(DcMotor.class, "front_claw");
        jewelArm   = hwMap.get(DcMotor.class, "jewel_arm");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        dunkClawArm.setDirection(DcMotor.Direction.REVERSE);
        slideClaw.setDirection(DcMotor.Direction.FORWARD);
        jewelArm.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftSlide.setPower(0);
        rightSlide.setPower(0);
        dunkClawArm.setPower(0);
        slideClaw.setPower(0);
        jewelArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dunkClawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideClaw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jewelArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jewelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dunkClawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        dunkClawLeft1  = hwMap.get(Servo.class, "left_back_claw");
        dunkClawRight1 = hwMap.get(Servo.class, "right_back_claw");
        dunkClawLeft1.setPosition(0.25);
        dunkClawRight1.setPosition(0.8);
        dunkClawLeft2  = hwMap.get(Servo.class, "left_back_claw_2");
        dunkClawRight2 = hwMap.get(Servo.class, "right_back_claw_2");
        dunkClawLeft2.setPosition(0.25);
        dunkClawRight2.setPosition(0.8);
    }
}
