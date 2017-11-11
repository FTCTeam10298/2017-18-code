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
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.OurM0Bot_Hardware;

/*
 * This is our TeleOp for Meet 0.
 */

@TeleOp(name="OurM0Bot: TeleOp", group="OurM0Bot")
public class OurM0Bot_TeleOp extends OpMode {

    /* Declare OpMode members. */
    OurM0Bot_Hardware robot       = new OurM0Bot_Hardware(); // use the class created to define OurM0Bot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          CLAW_OFFSET = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.01 ;                 // sets rate to move servo

    boolean togglePressed = false;
    boolean frontAndBackSwitched = false;

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
        double left;
        double right;

        if (gamepad1.dpad_down) {
            togglePressed = true;
        }
        else if (togglePressed) {
            frontAndBackSwitched = !frontAndBackSwitched; // Inverse "frontAndBackSwitched"
        }

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        if (!frontAndBackSwitched) {
            left  = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
        }
        else { // Drive as if the back is the front
            left  =  gamepad1.right_stick_y;
            right =  gamepad1.left_stick_y;
        }

        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            CLAW_OFFSET += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            CLAW_OFFSET -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        CLAW_OFFSET = Range.clip(CLAW_OFFSET, -0.5, 0.5);
        robot.leftBackClaw.setPosition(robot.MID_SERVO + CLAW_OFFSET);
        robot.rightBackClaw.setPosition(robot.MID_SERVO - CLAW_OFFSET);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y) {
            robot.leftArm.setPower(robot.ARM_UP_POWER);
            robot.rightArm.setPower(robot.ARM_UP_POWER);
        }
        else if (gamepad1.a) {
            robot.leftArm.setPower(robot.ARM_DOWN_POWER);
            robot.rightArm.setPower(robot.ARM_DOWN_POWER);
        }
        else {
            robot.leftArm.setPower(0.0);
            robot.rightArm.setPower(0.0);
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", CLAW_OFFSET);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
