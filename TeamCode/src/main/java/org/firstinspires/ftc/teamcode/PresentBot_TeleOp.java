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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/*
 * This is our secondary bot's TeleOp.
 */

@TeleOp(name="PresentBot: TeleOp", group="PresentBot")
public class PresentBot_TeleOp extends OpMode {

    /* Declare OpMode members. */
    PresentBot_Hardware robot = new PresentBot_Hardware(); // Use the class created to define OurBot's hardware
    double CLAW_SPEED  = 0.01;                // Sets rate to move servo
    double CLAW_OFFSET = 0.5;                // Offset from the servo's mid position

    double left        = 0.0;
    double right       = 0.0;
    double armUp       = 0.0;
    double armDown     = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Robot", "Initialized");
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            left = 1;
            right = 1;
        } else if (gamepad1.dpad_down) {
            left = -1;
            right = -1;
        } else if (gamepad1.dpad_left) {
            left = -1;
            right = 1;
        } else if (gamepad1.dpad_right) {
            left = 1;
            right = -1;
        } else {
            left = -(gamepad1.left_stick_y);
            left = left * Math.abs(left);
            right = -(gamepad1.right_stick_y);
            right = right * Math.abs(right);
        }

        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            CLAW_OFFSET += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            CLAW_OFFSET -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        CLAW_OFFSET = Range.clip(CLAW_OFFSET, -0.45, 0.45);
        robot.pivotClaw1.setPosition(0.5 - CLAW_OFFSET);
        robot.pivotClaw2.setPosition(0.5 + CLAW_OFFSET);

        // Use gamepad buttons to move the dunk claw up (left_trigger) and down (right_trigger)
        armUp = gamepad1.left_trigger;
        armDown = -gamepad1.right_trigger;
        armUp *= Math.abs(armUp);
        armDown *= Math.abs(armDown);
        if (armUp > 0.1) {
            robot.pivotArm.setPower(armUp/2);
        } else if (armDown < 0.1){
            robot.pivotArm.setPower(armDown/2);
        }
        else {
            robot.pivotArm.setPower(0.1);
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",   "Offset = %.2f", CLAW_OFFSET);
        telemetry.addData("left",   "%.2f",          left);
        telemetry.addData("right",  "%.2f",          right);
        telemetry.addData("armUp",  "%.2f",          armUp);
        telemetry.addData("arDown", "%.2f",          armDown);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
