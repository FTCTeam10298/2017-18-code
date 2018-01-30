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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FutureBot Teleop", group="FutureBot")
//@Disabled
public class FutureBot_Teleop extends OpMode {

    /* Declare OpMode members. */
    FutureBot_Hardware robot = new FutureBot_Hardware(); // use the class created to define Ourbot's hardware

    double   x = 0;
    double   y = 0;
    double   z = 0;

    double   CLAW_SPEED    = 0.01 ;                 // Sets rate to move servo
    double   CLAW_OFFSET_1 = 0.0 ;                  // Offset from the servo's mid position
    double   CLAW_OFFSET_2 = 0.0 ;                  // Offset from the servo's mid position

    double   RELIC_ELBOW_POSITION = 0.0 ;           // Offset from the servo's mid position
    double   RELIC_CLAW_POSITION  = 0.0 ;           // Offset from the servo's mid position

    double   arm           = 0.0;

    boolean  togglePressed        = false;
    boolean  toggle2Pressed       = false;
    boolean  frontAndBackSwitched = false;
    double   spinnyPosition       = 1;

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
//        robot.backLeftMotor.setPower(1);
//        sleep(1000);
//        robot.backRightMotor.setPower(1);
//        sleep(1000);
//        robot.frontLeftMotor.setPower(1);
//        sleep(1000);
//        robot.frontRightMotor.setPower(1);
//        sleep(1000);
//        robot.backLeftMotor.setPower(0);
//        sleep(1000);
//        robot.backRightMotor.setPower(0);
//        sleep(1000);
//        robot.frontLeftMotor.setPower(0);
//        sleep(1000);
//        robot.frontRightMotor.setPower(0);
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
        telemetry.addData("Say", "Running");
/*
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
*/
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
/*
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
*//*
        else if (gamepad1.right_trigger > 0.1) {
            TurnRight = gamepad1.right_trigger;
            robot.frontLeftMotor.setPower(TurnRight);
            robot.backLeftMotor.setPower(TurnRight);
            robot.frontRightMotor.setPower(-TurnRight);
            robot.backRightMotor.setPower(-TurnRight);
        }
        else if (gamepad1.left_trigger > 0.1) {
            TurnLeft = gamepad1.left_trigger;
            robot.frontLeftMotor.setPower(-TurnLeft);
            robot.backLeftMotor.setPower(-TurnLeft);
            robot.frontRightMotor.setPower(TurnLeft);
            robot.backRightMotor.setPower(TurnLeft);
        }
*//*
        // Enhanced tank drive
        else {
            leftFrontPower = Range.clip(-gamepad1.left_stick_y + (1 * gamepad1.left_stick_x), -1.0, 1.0);
            leftBackPower = Range.clip(-gamepad1.left_stick_y + (-1 * gamepad1.left_stick_x), -1.0, 1.0);
            rightFrontPower = Range.clip(-gamepad1.right_stick_y + (-1 * gamepad1.right_stick_x), -1.0, 1.0);
            rightBackPower = Range.clip(-gamepad1.right_stick_y + (1 * gamepad1.right_stick_x), -1.0, 1.0);

            robot.frontLeftMotor.setPower(leftFrontPower);
            robot.backLeftMotor.setPower(leftBackPower);
            robot.frontRightMotor.setPower(rightFrontPower);
            robot.backRightMotor.setPower(rightBackPower);
        } // End enhanced tank drive
*/
        // Drone drive
//        else {
            if (gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1) {
                y = gamepad1.left_stick_y;
            } else {
                y = 0;
            }

            if (gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < -.1) {
                x = gamepad1.left_stick_x;
            } else {
                x = 0;
            }

            if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1) {
                z = -gamepad1.right_stick_x/2;
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
            if (maxvalue < 1.0) {
                maxvalue = 1;
            }

            robot.frontRightMotor.setPower(-1 * Range.clip(((y + x - z) / maxvalue), -1.0, 1.0));
            robot.frontLeftMotor.setPower(-1 * Range.clip(((y - x + z) / maxvalue), -1.0, 1.0));
            robot.backLeftMotor.setPower(-1 * Range.clip(((y + x + z) / maxvalue), -1.0, 1.0));
            robot.backRightMotor.setPower(-1 * Range.clip(((y - x - z) / maxvalue), -1.0, 1.0));
//        } // End drone drive
/*
        // Standard tank drive
        else {
            robot.frontRightMotor.setPower(-gamepad1.right_stick_y);
            robot.frontLeftMotor.setPower(-gamepad1.left_stick_y);
            robot.backLeftMotor.setPower(-gamepad1.left_stick_y);
            robot.backRightMotor.setPower(-gamepad1.right_stick_y);
        }*/

        // Start glyph control ---------------------------------------------------------------------

        // Use gamepad left & right Bumpers to open and close the claw
        if (spinnyPosition == 1) {
            if (gamepad1.right_bumper || gamepad2.right_bumper)
                CLAW_OFFSET_1 += CLAW_SPEED;
            else if (gamepad1.left_bumper || gamepad2.left_bumper)
                CLAW_OFFSET_1 -= CLAW_SPEED;

            if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1)
                CLAW_OFFSET_2 += CLAW_SPEED;
            else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1)
                CLAW_OFFSET_2 -= CLAW_SPEED;
        }
        else {
            if (gamepad1.right_bumper || gamepad2.right_bumper)
                CLAW_OFFSET_2 += CLAW_SPEED;
            else if (gamepad1.left_bumper || gamepad2.left_bumper)
                CLAW_OFFSET_2 -= CLAW_SPEED;

            if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1)
                CLAW_OFFSET_1 += CLAW_SPEED;
            else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1)
                CLAW_OFFSET_1 -= CLAW_SPEED;
        }

        // Move both servos to new position.  Assume servos are mirror image of each other.
        CLAW_OFFSET_1 = Range.clip(CLAW_OFFSET_1, -0.15, 0.30);
        robot.dunkClawLeft1.setPosition(0.5 - CLAW_OFFSET_1);
        robot.dunkClawRight1.setPosition(0.5 + CLAW_OFFSET_1);
        CLAW_OFFSET_2 = Range.clip(CLAW_OFFSET_2, -0.15, 0.30);
        robot.dunkClawLeft2.setPosition(0.5 - CLAW_OFFSET_2);
        robot.dunkClawRight2.setPosition(0.5 + CLAW_OFFSET_2);

        // spinny claw
        if (gamepad1.dpad_left || gamepad2.dpad_left || gamepad1.dpad_right || gamepad2.dpad_right) {
//        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            toggle2Pressed = true;
        }
        else if (toggle2Pressed) {
            if (spinnyPosition == 0)
                spinnyPosition = 1;
            else
                spinnyPosition = 0;
            toggle2Pressed = false;
        }

        robot.spinnyClaw.setPosition(spinnyPosition);
/*
        // Use gamepad buttons to move the slides up (Y) and down (A)
        if (gamepad1.y || gamepad2.y) {
            robot.wallSlide.setPower(0.6);
        }
        else if (gamepad1.a || gamepad2.a) {
            robot.wallSlide.setPower(-0.5);
        }
        else {
            robot.wallSlide.setPower(0.0);
        }
*/
        // Use gamepad buttons to move the dunk claw up (DPAD_UP) and down (DPAD_DOWN)
//        arm = gamepad2.right_stick_y;
//        arm = arm*Math.abs(arm);
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
//        if (gamepad2.dpad_up) {
            robot.dunkClawArm.setPower(1.0);
        }
        else if (gamepad1.dpad_down || gamepad2.dpad_down) {
//        else if (gamepad2.dpad_down) {
            robot.dunkClawArm.setPower(-1.0);
        }
//        else if (arm > 0.1 || arm < -0.1) {
//            robot.dunkClawArm.setPower(arm);
//        }
        else {
            robot.dunkClawArm.setPower(0.0);
        }

        // Move the jewel arm so it doesn't get in the way
        if (gamepad2.start) {
            robot.jewelArm.setPower(0.3);
        }
        else if (gamepad2.left_stick_button) {
            robot.jewelArm.setPower(-0.3);
        }
        else {
            robot.jewelArm.setPower(0);
        }

        // Start Relic control ---------------------------------------------------------------------

        robot.relicLift.setPower(Range.clip(-gamepad2.left_stick_y+0.001,0,1));
        robot.relicOut.setPower(-gamepad2.right_stick_y);

        // Use gamepad buttons to move the elbow up (Y) and down (X)
        if (gamepad1.y || gamepad2.y)
            RELIC_ELBOW_POSITION += CLAW_SPEED/2;
        else if (gamepad1.x || gamepad2.x)
            RELIC_ELBOW_POSITION -= CLAW_SPEED/2;

        // Use gamepad buttons to open (B) and close (A) the claw
        if (gamepad1.b || gamepad2.b)
            RELIC_CLAW_POSITION += CLAW_SPEED/2;
        else if (gamepad1.a || gamepad2.a)
            RELIC_CLAW_POSITION -= CLAW_SPEED/2;

        // Move both servos to new position.
        RELIC_ELBOW_POSITION = Range.clip(RELIC_ELBOW_POSITION, -1.0, 1.0);
        robot.relicElbow.setPosition(RELIC_ELBOW_POSITION);
        RELIC_CLAW_POSITION = Range.clip(RELIC_CLAW_POSITION, -0.5, 0.5);
        robot.relicClaw.setPosition(RELIC_CLAW_POSITION);

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", CLAW_OFFSET_1);
        //telemetry.addData("inertia", "%.2f", inertia);
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
