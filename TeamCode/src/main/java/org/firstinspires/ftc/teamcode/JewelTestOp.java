/*
Copyright (c) 2018, FTC team #10298 Brain Stormz

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Brain Stormz nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import hallib.HalDashboard;

import static java.lang.Math.abs;

/**
 * This file provides Teleop driving for our robot.
 * The code is structured as an Iterative OpMode.
 *
 * This OpMode uses the common FutureBot hardware class to define the devices on the robot.
 * All device access is managed through the FutureBot_Hardware class.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="JewelTestOp", group="FutureBot")

public class JewelTestOp extends OpMode {

    /* Declare OpMode members. */
    private HalDashboard dashboard;
    FutureBot_Hardware robot = new FutureBot_Hardware(); // use the class created to define FutureBot's hardware

    double   jewelArmPosition = .5;

    double   jewelKnock       = .5;

    double   INTAKE_OFFSET    = 0.0 ;                  // Offset from the servo's mid position

    double   spinnyPosition   = 0;
    double   jewelPosition    = .5;

    int      state            = 0;
    int      count            = 0;

    // Code to run once when the driver hits INIT
    @Override
    public void init() {
        /*
         * Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        dashboard = HalDashboard.createInstance(telemetry);

    }

    /*
     * Code to run in a loop after the driver hits play until they hit the stop button
     */
    @Override
    public void loop() {

        if (gamepad1.dpad_up)
           jewelArmPosition += .001;
        else if (gamepad1.dpad_down)
           jewelArmPosition -= .001;

        jewelArmPosition = Range.clip(jewelArmPosition, 0, 1);//.366

        if (gamepad1.dpad_left)
           jewelPosition += .001;
        else if (gamepad1.dpad_right)
           jewelPosition -= .001;

        if (gamepad1.a)
           jewelPosition = 0.5 + jewelKnock;
        else if (gamepad1.b)
           jewelPosition = 0.5 - jewelKnock;

        jewelPosition = Range.clip(jewelPosition, 0, 1);

        if (gamepad1.x)
           jewelKnock += .001;
        else if (gamepad1.y)
           jewelKnock -= .001;

        jewelKnock = Range.clip(jewelKnock, 0, 0.5);

        if (gamepad1.right_bumper)
            jewelPosition = 0.5;

        if (gamepad1.left_bumper)
            jewelArmPosition = 1;

        robot.jewelHitter.setPosition(jewelPosition);
        robot.jewelArm.setPosition(jewelArmPosition);

        dashboard.displayPrintf(0, "jewelArmPosition " + jewelArmPosition);
        dashboard.displayPrintf(1, "jewelPosition " + jewelPosition);
        dashboard.displayPrintf(2, "jewelKnock " + jewelKnock);

//        for (double i = .5; i >= .3; i -= .02) {
//            robot.jewelHitter.setPosition(i);
//        }

    }

    /*
    FUNCTIONS------------------------------------------------------------------------------------------------------
     */
    void DrivePowerAll (double power)
    {
        robot.frontLeftDrive.setPower(power);
        robot.frontRightDrive.setPower(power);
        robot.backRightDrive.setPower(power);
        robot.backLeftDrive.setPower(power);
    }

    void DriveRobotTurn (double power)
    {
        robot.frontLeftDrive.setPower(-power);
        robot.frontRightDrive.setPower(power);
        robot.backRightDrive.setPower(power);
        robot.backLeftDrive.setPower(-power);
    }

    void DriveSideways (double power)
    {
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (power > 0) // Drive right
        {
            robot.frontLeftDrive.setPower(-power);
            robot.backLeftDrive.setPower(power);
            robot.backRightDrive.setPower(-power);
            robot.frontRightDrive.setPower(power);
        }
        else // Drive left
        {
            robot.frontRightDrive.setPower(power);
            robot.backRightDrive.setPower(-power);
            robot.backLeftDrive.setPower(power);
            robot.frontLeftDrive.setPower(-power);
        }
    }
    void LiftSpinDrop () {
        if (state == 0) {
            count = 0;
        } else if (state == 1) {
            robot.dunkClawArm.setPower(1);
            robot.dunkClawArm.setTargetPosition(8000);
            if (robot.dunkClawArm.getCurrentPosition() > 800) {
                robot.dunkClawArm.setPower(0);
                state = 2;
            }
        } else if (state == 2) {
            if (spinnyPosition == 0)
                spinnyPosition = 1;
            else
                spinnyPosition = 0;
            state = 3;
        } else if (state == 3) {
            count++;
            if (count == 180) {
                count = 0;
                state = 4;
            }
        } else if (state == 4) {
            robot.dunkClawArm.setPower(1);
            robot.dunkClawArm.setTargetPosition(-100);
            if (robot.dunkClawArm.getCurrentPosition() < 0) {
                robot.dunkClawArm.setPower(0);
                robot.dunkClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.dunkClawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                INTAKE_OFFSET += 0.15;
                state = 0;
            }
        }
    }
}
