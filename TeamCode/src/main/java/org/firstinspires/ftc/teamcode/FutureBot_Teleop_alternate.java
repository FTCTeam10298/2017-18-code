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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

/**
 * This file provides Teleop driving for our robot.
 * The code is structured as an Iterative OpMode.
 *
 * This OpMode uses the common FutureBot hardware class to define the devices on the robot.
 * All device access is managed through the FutureBot_Hardware class.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FutureBot Alternate TeleOp", group="FutureBot")
@Disabled
public class FutureBot_Teleop_alternate extends OpMode {

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

    boolean  toggle2Pressed       = false;
    boolean  toggle3Pressed       = false;
    boolean  toggle4Pressed       = false;
    double   spinnyPosition       = 1;

    int      state                = 0;
    int      count                = 0;

    int      intake               = 0;
    int      count2               = 0;

    boolean  logitech1            = false;
    boolean  logitech2            = true;
    int      divide1;
    int      divide2;
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


        // Send telemetry message to signify robot running
        telemetry.addData("Say", "Running");

        // Logitech
/*
            if (gamepad1.guide) {
                if (logitech1)
                    logitech1 = false;
                else
                    logitech1 = true;
            }
            if (gamepad2.guide) {
                if (logitech2)
                    logitech2 = false;
                else
                    logitech2 = true;
            }

            if (divide1 == 1 && logitech1)
                divide1 = 2;
            else if (divide1 == 2 && !logitech1)
                divide1 = 1;

            if (divide2 == 1 && logitech2)
                divide2 = 2;
            else if (divide2 == 2 && !logitech2)
                divide2 = 1;
*/
        // Drone drive

            if (gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1) {
                y = gamepad1.left_stick_y; // divide1;
            }
            else if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) {
                    y = gamepad2.left_stick_y; // divide2;
            } else {
                y = 0;
            }

            if (gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < -.1) {
                x = gamepad1.left_stick_x; // divide1;
            }
            else if (gamepad2.left_stick_x > .1 || gamepad2.left_stick_x < -.1) {
                x = gamepad2.left_stick_x; // divide2;
            } else {
                x = 0;
            }

            if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1) {
                z = -gamepad1.right_stick_x / 2;// * divide1;
            }
            else if (gamepad2.right_stick_x > .2 || gamepad2.right_stick_x < -.2) {
                z = -gamepad2.right_stick_x / 4;// * divide2;
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
            // End drone drive



        // Start glyph control ---------------------------------------------------------------------

        // Use gamepad left & right Bumpers to open and close the claw
        if (spinnyPosition == 1) {
            if (gamepad1.right_bumper)
                CLAW_OFFSET_1 += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                CLAW_OFFSET_1 -= CLAW_SPEED;

            if (gamepad1.right_trigger > 0.1)
                CLAW_OFFSET_2 += CLAW_SPEED;
            else if (gamepad1.left_trigger > 0.1)
                CLAW_OFFSET_2 -= CLAW_SPEED;
        }
        else {
            if (gamepad1.right_bumper)
                CLAW_OFFSET_2 += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                CLAW_OFFSET_2 -= CLAW_SPEED;

            if (gamepad1.right_trigger > 0.1)
                CLAW_OFFSET_1 += CLAW_SPEED;
            else if (gamepad1.left_trigger > 0.1)
                CLAW_OFFSET_1 -= CLAW_SPEED;
        }

        // Move both servos to new position.  Assume servos are mirror image of each other.
        CLAW_OFFSET_1 = Range.clip(CLAW_OFFSET_1, -0.5, 0.5);
        robot.dunkClawRight1.setPosition(0.5 + CLAW_OFFSET_1);
        CLAW_OFFSET_2 = Range.clip(CLAW_OFFSET_2, -0.5, 0.5);
        robot.dunkClawRight2.setPosition(0.5 + CLAW_OFFSET_2);

        // spinny claw
        if (gamepad1.dpad_left || gamepad1.dpad_right) {
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

        // Use a (in) and b (out) to move intake
        if (gamepad1.a && !toggle3Pressed) {
            if (intake == 1) {
                intake = 0;
            } else {
                intake = 1;
            }
            toggle3Pressed = true;
        }
        else if (gamepad1.b && !toggle4Pressed) {
            if (intake == 2) {
                intake = 0;
            } else {
                intake = 2;
            }
            toggle4Pressed = true;
        }
        else {
            toggle3Pressed = false;
            toggle4Pressed = false;
        }

        if (intake == 0) {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
        else if (intake == 1){
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(-1);
        }
        else if (intake == 2){
            robot.leftIntake.setPower(1);
            robot.rightIntake.setPower(1);
        }

        count2++;

        // Use gamepad buttons to move the dunk claw up (DPAD_UP) and down (DPAD_DOWN)
        if (gamepad1.dpad_up) {
            robot.dunkClawArm.setPower(1.0);
        }
        else if (gamepad1.dpad_down) {
            robot.dunkClawArm.setPower(-1.0);
        }
        else {
            robot.dunkClawArm.setPower(0.0);
        }

        // Lift, Spin, Drop
        if (gamepad1.x && state == 0) {
            state = 1;
            robot.dunkClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.dunkClawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.dunkClawArm.setPower(0);
        }
        else if (state > 0 && (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left
                || gamepad1.dpad_right || gamepad1.left_bumper || gamepad1.right_bumper
                || (gamepad1.right_trigger > .1) || (gamepad1.left_trigger > .1))) {
            state = 0;
            LiftSpinDrop();
            robot.dunkClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.dunkClawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            LiftSpinDrop();
        }

        // Move the jewel arm so it doesn't get in the way
//        if (gamepad2.right_stick_button) {
//            robot.jewelArm.setPower(0.3);
//        }
//        else if (gamepad2.left_stick_button) {
//            robot.jewelArm.setPower(-0.3);
//        }

        // Start Relic control ---------------------------------------------------------------------

        // Use gamepad stick y to extend relic arm
        if (gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < .1) {
            robot.relicOut.setPower(gamepad2.right_stick_y);
        }

        // Use gamepad dpad up and down to raise and lower relic arm
        if (gamepad2.dpad_up) {
            robot.relicLift.setPower(1);
        } else if (gamepad2.dpad_down) {
            robot.relicLift.setPower(-1);
        } else {
            robot.relicLift.setPower(0.01);
        }

        // Use gamepad dpad to move the elbow up (right) and down (left)
        if (gamepad2.dpad_right)
            RELIC_ELBOW_POSITION += CLAW_SPEED/1.5;
        else if (gamepad2.dpad_left)
            RELIC_ELBOW_POSITION -= CLAW_SPEED/1.5;

        // Use gamepad bumpers to open (left) and close (right) the claw
        if (gamepad2.right_bumper)
            RELIC_CLAW_POSITION += CLAW_SPEED/2;
        else if (gamepad2.left_bumper)
            RELIC_CLAW_POSITION -= CLAW_SPEED/2;

        // Move both servos to new position.
        RELIC_ELBOW_POSITION = Range.clip(RELIC_ELBOW_POSITION, -1.0, 1.0);
        robot.relicElbow.setPosition(RELIC_ELBOW_POSITION);
        RELIC_CLAW_POSITION = Range.clip(RELIC_CLAW_POSITION, -0.5, 0.5);
        robot.relicClaw.setPosition(RELIC_CLAW_POSITION);

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", CLAW_OFFSET_1);
        telemetry.addData("count", count);
        telemetry.addData("state", state);


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

    void LiftSpinDrop ()
    {
        if (state == 0) {
            count = 0;
        }
        else if (state == 1){
            robot.dunkClawArm.setPower(1);
            robot.dunkClawArm.setTargetPosition(1800);
            if (!robot.dunkClawArm.isBusy()) {
                robot.dunkClawArm.setPower(0);
                state = 2;
            }
        }
        else if (state == 2) {
            if (spinnyPosition == 0)
                spinnyPosition = 1;
            else
                spinnyPosition = 0;
            state = 3;
        }
        else if (state == 3) {
            count ++;
            if (count == 150) {
                count = 0;
                state = 4;
            }
        }
        else if (state == 4){
            robot.dunkClawArm.setPower(1);
            robot.dunkClawArm.setTargetPosition(0);
            if (!robot.dunkClawArm.isBusy()) {
                robot.dunkClawArm.setPower(0);
                robot.dunkClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.dunkClawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                state = 0;
            }
        }
    }
}
