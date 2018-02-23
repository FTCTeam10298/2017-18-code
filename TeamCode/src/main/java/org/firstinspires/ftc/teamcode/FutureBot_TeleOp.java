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
import static java.lang.Math.abs;

/**
 * This file provides Teleop driving for our robot.
 * The code is structured as an Iterative OpMode.
 *
 * This OpMode uses the common FutureBot hardware class to define the devices on the robot.
 * All device access is managed through the FutureBot_Hardware class.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FutureBot TeleOp", group="FutureBot")
public class FutureBot_TeleOp extends OpMode {

    /* Declare OpMode members. */
    FutureBot_Hardware robot = new FutureBot_Hardware(); // use the class created to define FutureBot's hardware

    double   x = 0;
    double   y = 0;
    double   z = 0;

    double   CLAW_SPEED        = 0.01;              // Sets the rate to move the servos
    double   CLAW_OFFSET_1     = 0.0 ;              // Offset from the servo's mid position
    double   CLAW_OFFSET_2     = 0.0 ;              // Offset from the servo's mid position
    double   INTAKE_OFFSET     = 0.0 ;              // Offset from the servo's mid position
    double   INTAKE_CORRECTION = 0.0 ;

    double   RELIC_ELBOW_POSITION = 0.0;           // Offset from the servo's mid position
    double   RELIC_CLAW_POSITION  = 0.0;           // Offset from the servo's mid position

    boolean  spinTogglePressed      = false;
    boolean  intakeInTogglePressed  = false;
    boolean  intakeOutTogglePressed = false;
    boolean  gamepad2ModeToggle     = false;
    double   spinnyPosition         = 0;
    int      jewelPosition          = 1;

    int      state     = 0;
    int      count     = 0;
    int      intake    = 0;

    boolean  glyph     = true;

    int      downCount = 0;

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
//        robot.backLeftDrive.setPower(1);
//        sleep(1000);
//        robot.backRightDrive.setPower(1);
//        sleep(1000);
//        robot.frontLeftDrive.setPower(1);
//        sleep(1000);
//        robot.frontRightDrive.setPower(1);
//        sleep(1000);
//        robot.backLeftDrive.setPower(0);
//        sleep(1000);
//        robot.backRightDrive.setPower(0);
//        sleep(1000);
//        robot.frontLeftDrive.setPower(0);
//        sleep(1000);
//        robot.frontRightDrive.setPower(0);
    }

    /*
     * Code to run in a loop after the driver hits play until they hit the stop button
     */
    @Override
    public void loop() {

        // Send telemetry message to signify robot running
        //telemetry.addData("Say", "Running");
        if (!glyph && (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1)){
            DriveSideways(gamepad2.left_stick_y/2);
        }
        else if (!glyph && (gamepad2.left_stick_x < -.1 || gamepad2.left_stick_x > .1)){
            DrivePowerAll(-gamepad2.left_stick_x/4);
        }
        else if (!glyph && (gamepad2.right_stick_x > .1 || gamepad2.right_stick_x < -.1)){
            DriveRobotTurn(-gamepad2.right_stick_x/4);
        }
        else if (gamepad1.dpad_up){
            DrivePowerAll(.5);
        }
        else if (gamepad1.dpad_down){
            DrivePowerAll(-.5);
        }
        else if (gamepad1.dpad_left){
            DriveSideways(.5);
        }
        else if (gamepad1.dpad_right){
            DriveSideways(-.5);
        }
        // Drone drive
        else {
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
                z = -gamepad1.right_stick_x / 2;
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

            robot.frontRightDrive.setPower(-1 * Range.clip(((y + x - z) / maxvalue), -1.0, 1.0));
            robot.frontLeftDrive.setPower(-1 * Range.clip(((y - x + z) / maxvalue), -1.0, 1.0));
            robot.backLeftDrive.setPower(-1 * Range.clip(((y + x + z) / maxvalue), -1.0, 1.0));
            robot.backRightDrive.setPower(-1 * Range.clip(((y - x - z) / maxvalue), -1.0, 1.0));
        }

        // Toggle between glyph and relic control
        if (gamepad2.y) {
            gamepad2ModeToggle = true;
        } else if (gamepad2ModeToggle) {
            glyph = !glyph;
            gamepad2ModeToggle = false;
        }

        // Start intake controls -------------------------------------------------------------------

        // Use gamepad left & right triggers to open and close the intake
        if (gamepad1.right_trigger > 0.5)
            INTAKE_OFFSET += CLAW_SPEED;
        else if (gamepad1.left_trigger > 0.5)
            INTAKE_OFFSET -= CLAW_SPEED;

        // Use gamepad left & right triggers to open and close the intake
        if (gamepad1.dpad_right)
            INTAKE_CORRECTION += CLAW_SPEED;
        else if (gamepad1.dpad_left)
            INTAKE_CORRECTION -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        INTAKE_OFFSET = Range.clip(INTAKE_OFFSET, -0.5, 0);
        INTAKE_CORRECTION = Range.clip(INTAKE_CORRECTION, -0.15, 0.15);
        robot.intakeRotateRight.setPosition(Range.clip(0.5 - INTAKE_OFFSET + INTAKE_CORRECTION, 0, 1));
        robot.intakeRotateLeft.setPosition(Range.clip(0.5 + INTAKE_OFFSET + INTAKE_CORRECTION, 0, 1));

        // Use a (in) and b (out) to move intake
        if (gamepad1.left_bumper || gamepad1.a || (glyph && gamepad2.a)) {
            intakeInTogglePressed = true;
        } else if (gamepad1.right_bumper || gamepad1.b || (glyph && gamepad2.b)) {
            intakeOutTogglePressed = true;
        } else if (intakeInTogglePressed) {
            if (intake == 1) {
                intake = 0;
            } else {
                intake = 1;
            }
            intakeInTogglePressed = false;
        } else if (intakeOutTogglePressed) {
            if (intake == 2) {
                intake = 0;
            } else {
                intake = 2;
            }
            intakeOutTogglePressed = false;
        }

        if (intake == 0) {
            robot.IntakeLeft.setPower(0);
            robot.IntakeRight.setPower(0);
        } else if (intake == 1) {
            robot.IntakeLeft.setPower(.5);
            robot.IntakeRight.setPower(1);
        } else if (intake == 2) {
            robot.IntakeLeft.setPower(-.5);
            robot.IntakeRight.setPower(-1);
        }

        // Danny Al's x
        if (gamepad1.x)
            INTAKE_OFFSET = 0;

        /*
         * Start glyph control ---------------------------------------------------------------------
         */

        if (glyph) {
            // Use gamepad left & right Bumpers and triggers to open and close the claws
            if (spinnyPosition == 1) {
                if (gamepad2.right_bumper)
                    CLAW_OFFSET_1 += CLAW_SPEED;
                else if (gamepad2.left_bumper)
                    CLAW_OFFSET_1 -= CLAW_SPEED;

                if (gamepad2.right_trigger > 0.1)
                    CLAW_OFFSET_2 += CLAW_SPEED;
                else if (gamepad2.left_trigger > 0.1)
                    CLAW_OFFSET_2 -= CLAW_SPEED;
            } else {
                if (gamepad2.right_bumper)
                    CLAW_OFFSET_2 += CLAW_SPEED;
                else if (gamepad2.left_bumper)
                    CLAW_OFFSET_2 -= CLAW_SPEED;

                if (gamepad2.right_trigger > 0.1)
                    CLAW_OFFSET_1 += CLAW_SPEED;
                else if (gamepad2.left_trigger > 0.1)
                    CLAW_OFFSET_1 -= CLAW_SPEED;
            }

            // Move both servos to new position.  Assume servos are mirror image of each other.
            CLAW_OFFSET_1 = Range.clip(CLAW_OFFSET_1, -0.5, 0.5);
            robot.dunkClaw1.setPosition(0.5 + CLAW_OFFSET_1);
            CLAW_OFFSET_2 = Range.clip(CLAW_OFFSET_2, -0.5, 0.5);
            robot.dunkClaw2.setPosition(0.5 + CLAW_OFFSET_2);

            // spinny claw
            if (gamepad2.dpad_left || gamepad2.dpad_right) {
                spinTogglePressed = true;
            } else if (spinTogglePressed) {
                if (spinnyPosition == 0)
                    spinnyPosition = 1;
                else
                    spinnyPosition = 0;
                spinTogglePressed = false;
            }

            robot.spinnyClaw.setPosition(spinnyPosition);

            // Use gamepad buttons to move the dunk claw up (DPAD_UP) and down (DPAD_DOWN)
            if (gamepad2.dpad_up)
                robot.dunkClawArm.setPower(1.0);
            else if (gamepad2.dpad_down) {
                robot.dunkClawArm.setPower(-1.0);
                downCount ++;
            }
            else {
                robot.dunkClawArm.setPower(0.0);
                if (downCount > 0){
                    downCount = 0;
                    INTAKE_OFFSET = 0.5;
                }
            }

            // Move the intake out when down is pressed
            if (downCount == 100)
                INTAKE_OFFSET -= .15;


            // Move the jewel arm so it doesn't get in the way
            if (gamepad2.right_stick_button)
                jewelPosition += .05;
            else if (gamepad2.left_stick_button)
                jewelPosition -= .05;

            jewelPosition = Range.clip(jewelPosition, 0, 1);
            robot.jewelArm.setPosition(jewelPosition);

            // Lift, Spin, Drop
            if (gamepad2.x && state == 0) {
                state = 1;
                robot.dunkClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.dunkClawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.dunkClawArm.setPower(0);
                INTAKE_OFFSET -= 0.15;
            }
            else if (state > 0 && (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left
                    || gamepad2.dpad_right || gamepad2.left_bumper || gamepad2.right_bumper
                    || (gamepad2.right_trigger > .1) || (gamepad2.left_trigger > .1))) {
                state = 0;
                LiftSpinDrop();
                robot.dunkClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.dunkClawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                INTAKE_OFFSET += 0.15;
            } else {
                LiftSpinDrop();
            }
        }

        /*
         * Start Relic control ---------------------------------------------------------------------
         */

        else {
            if (gamepad2.right_trigger > 0.1)
                robot.relicOut.setPower(-gamepad2.right_trigger);
            else if (gamepad2.left_trigger > 0.1)
                robot.relicOut.setPower(gamepad2.left_trigger);
            else
                robot.relicOut.setPower(0);

            // Use gamepad buttons to move the elbow up (Right Trigger) and down (Left Trigger)
            if (gamepad2.dpad_right)
                RELIC_ELBOW_POSITION += CLAW_SPEED / 4;
            else if (gamepad2.dpad_left)
                RELIC_ELBOW_POSITION -= CLAW_SPEED / 4;
            else if (gamepad2.a)
                RELIC_ELBOW_POSITION = .65;

            // Use gamepad buttons to open (Left Bumper) and close (Right Bumper) the claw
            if (gamepad2.left_bumper)
                RELIC_CLAW_POSITION += CLAW_SPEED / 2;
            else if (gamepad2.right_bumper)
                RELIC_CLAW_POSITION -= CLAW_SPEED / 2;

            // Use gamepad dpad up and down to raise and lower relic arm
            if (gamepad2.dpad_up) {
                robot.relicLift.setPower(1);
            } else if (gamepad2.dpad_down) {
                robot.relicLift.setPower(-1);
            } else {
                robot.relicLift.setPower(0.01);
            }

            // Move both servos to new position.
            RELIC_ELBOW_POSITION = Range.clip(RELIC_ELBOW_POSITION, 0.0, 1.0);
            robot.relicElbow.setPosition(RELIC_ELBOW_POSITION);
            RELIC_CLAW_POSITION = Range.clip(RELIC_CLAW_POSITION, -0.5, 0.5);
            robot.relicClaw.setPosition(RELIC_CLAW_POSITION);
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", CLAW_OFFSET_1);
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
            robot.dunkClawArm.setTargetPosition(0);
            if (robot.dunkClawArm.getCurrentPosition() < 30) {
                robot.dunkClawArm.setPower(0);
                robot.dunkClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.dunkClawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                INTAKE_OFFSET += 0.15;
                state = 0;
            }
        }
    }
}
