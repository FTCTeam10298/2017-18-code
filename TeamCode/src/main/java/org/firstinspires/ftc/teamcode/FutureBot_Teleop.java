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

Neither the name of Robert Atkinson nor the names of his contributors may be used to
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

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file provides Teleop driving for our robot.
 * The code is structured as an Iterative OpMode.
 *
 * This OpMode uses the common FutureBot hardware class to define the devices on the robot.
 * All device access is managed through the FutureBot_Hardware class.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FutureBot_Teleop", group="FutureBot")
//@Disabled
public class FutureBot_Teleop extends OpMode {

    /* Declare OpMode members. */
    FutureBot_Hardware  robot       = new FutureBot_Hardware(); // use the class created to define Ourbot's hardware

    double              x           = 0;
    double              y           = 0;
    double              z           = 0;

    // Code to run once when the driver hits INIT
    @Override
    public void init() {
        /*
         * Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Robot ready");
    }

    /*
     * Code to run in a loop after the driver hits play until they hit the stop button
     */
    @Override
    public void loop() {

        double leftFrontPower;
        double leftBackPower;
        double rightBackPower;
        double rightFrontPower;

        double TurnLeft;
        double TurnRight;

        // Send telemetry message to signify robot running
        if (gamepad1.x) {
            telemetry.addData("Say", "Running");
        }

        // Gamepad 2 override
        if (gamepad2.left_stick_y < -0.1) {
            DrivePowerAll(0.25);
        }
        else if (gamepad2.left_stick_y > 0.1) {
            DrivePowerAll(-0.25);
        }
        else if (gamepad2.left_stick_x > 0.1) {
            DriveSideways(0.25);
        }
        else if (gamepad2.left_stick_x < -0.1) {
            DriveSideways(-0.25);
        } // End gamepad 2 override
      
        /* // START OF SIDE DRIVE
        double SideDriveL = gamepad1.left_trigger;
        double SideDriveR = gamepad1.right_trigger;
        if (SideDriveL > 0.1) {
            robot.frontLeftMotor.setPower(SideDriveL);
            robot.backLeftMotor.setPower(-SideDriveL);
            robot.frontRightMotor.setPower(-SideDriveL);
            robot.backRightMotor.setPower(SideDriveL);
        } else if (SideDriveR > 0.1) {
            robot.frontLeftMotor.setPower(-SideDriveR);
            robot.backLeftMotor.setPower(SideDriveR);
            robot.frontRightMotor.setPower(SideDriveR);
            robot.backRightMotor.setPower(-SideDriveR);
        }
        // END OF SIDE DRIVE
        */

        // START OF DPAD DRIVE
        else if (gamepad1.dpad_right) {
            robot.frontLeftMotor.setPower(1);
            robot.backLeftMotor.setPower(-1);
            robot.frontRightMotor.setPower(-1);
            robot.backRightMotor.setPower(1);
        }
        else if (gamepad1.dpad_left) {
            robot.frontLeftMotor.setPower(-1);
            robot.backLeftMotor.setPower(1);
            robot.frontRightMotor.setPower(1);
            robot.backRightMotor.setPower(-1);
        }
        else if (gamepad1.dpad_down) {
            robot.frontLeftMotor.setPower(-1);
            robot.backLeftMotor.setPower(-1);
            robot.frontRightMotor.setPower(-1);
            robot.backRightMotor.setPower(-1);
        }
        else if (gamepad1.dpad_up) {
            TurnRight = gamepad1.right_trigger;
            TurnLeft = gamepad1.left_trigger;

            if (TurnRight > 0.1) {
                robot.frontLeftMotor.setPower(1);
                robot.backLeftMotor.setPower(1);
                robot.frontRightMotor.setPower(1 - TurnRight);
                robot.backRightMotor.setPower(1 - TurnRight);
            }
            else if (TurnLeft > 0.1) {
                robot.frontLeftMotor.setPower(1 - TurnLeft);
                robot.backLeftMotor.setPower(1 - TurnLeft);
                robot.frontRightMotor.setPower(1);
                robot.backRightMotor.setPower(1);
            }
            else {
                robot.frontLeftMotor.setPower(1);
                robot.backLeftMotor.setPower(1);
                robot.frontRightMotor.setPower(1);
                robot.backRightMotor.setPower(1);
            }
        }
        // END OF DPAD DRIVE

        else if (gamepad1.right_bumper) {
                robot.frontLeftMotor.setPower (1);
                robot.backLeftMotor.setPower (1);
                robot.frontRightMotor.setPower (-1);
                robot.backRightMotor.setPower (-1);
        }
        else if (gamepad1.left_bumper) {
                robot.frontLeftMotor.setPower (-1);
                robot.backLeftMotor.setPower (-1);
                robot.frontRightMotor.setPower (1);
                robot.backRightMotor.setPower (1);
        }
/*
        // Enhanced tank drive
        else {
            leftFrontPower = Range.clip(gamepad1.left_stick_y + (-1 * gamepad1.left_stick_x), -1.0, 1.0);
            leftBackPower = Range.clip(gamepad1.left_stick_y + (1 * gamepad1.left_stick_x), -1.0, 1.0);
            rightFrontPower = Range.clip(gamepad1.right_stick_y + (1 * gamepad1.right_stick_x), -1.0, 1.0);
            rightBackPower = Range.clip(gamepad1.right_stick_y + (-1 * gamepad1.right_stick_x), -1.0, 1.0);

            robot.frontLeftMotor.setPower(leftFrontPower);
            robot.backLeftMotor.setPower(leftBackPower);
            robot.frontRightMotor.setPower(rightFrontPower);
            robot.backRightMotor.setPower(rightBackPower);
        } // End enhanced tank drive
*//*
        // Drone drive
        else {
            if (gamepad1.left_stick_y > .2 || gamepad1.left_stick_y < -.2) {
                y = gamepad1.left_stick_y;
            } else {
                y = 0;
            }

            if (gamepad1.left_stick_x > .2 || gamepad1.left_stick_x < -.2) {
                x = gamepad1.left_stick_x;
            } else {
                x = 0;
            }

            if (gamepad1.right_stick_x > .2 || gamepad1.right_stick_x < -.2) {
                z = -gamepad1.right_stick_x;
            } else {
                z = 0;
            }

            double maxvalue = abs(y + x - z);
            if (abs(y + x - z) > maxvalue) {
                maxvalue = abs(y + x - z);
            }
            if (abs(y - x + z) > maxvalue) {
                maxvalue = abs(y - x + z);
            }
            if (abs(y + x + z) > maxvalue) {
                maxvalue = abs(y + x + z);
            }
            if (abs(y - x - z) > maxvalue) {
                maxvalue = abs(y - x - z);
            }
            if (maxvalue == 0) {
                maxvalue = 1;
            }

            robot.frontRightMotor.setPower(-1 * Range.clip(((y + x - z) / maxvalue), -1.0, 1.0));
            robot.frontLeftMotor.setPower(-1 * Range.clip(((y - x + z) / maxvalue), -1.0, 1.0));
            robot.backLeftMotor.setPower(-1 * Range.clip(((y + x + z) / maxvalue), -1.0, 1.0));
            robot.backRightMotor.setPower(-1 * Range.clip(((y - x - z) / maxvalue), -1.0, 1.0));
        } // End drone drive
*/
        // Standard tank drive
        else {
            robot.frontRightMotor.setPower(-gamepad1.right_stick_y);
            robot.frontLeftMotor.setPower(-gamepad1.left_stick_y);
            robot.backLeftMotor.setPower(-gamepad1.left_stick_y);
            robot.backRightMotor.setPower(-gamepad1.right_stick_y);
        }
    }

    @Override
    public void stop () {
        // Code here runs ONCE after the driver hits stop

    }

    /*
    FUNCTIONS------------------------------------------------------------------------------------------------------
     */
    void DrivePowerAll (double power)
    {
        robot.frontLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.backLeftMotor.setPower(power);
    }

    void DriveSideways (double power)
    {
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (power > 0) // Drive right
        {
            robot.frontLeftMotor.setPower(-power);
            robot.backLeftMotor.setPower(power);
            robot.backRightMotor.setPower(-power);
            robot.frontRightMotor.setPower(power);
        }
        else // Drive left
        {
            robot.frontRightMotor.setPower(power);
            robot.backRightMotor.setPower(-power);
            robot.backLeftMotor.setPower(power);
            robot.frontLeftMotor.setPower(-power);
        }
    }
}
