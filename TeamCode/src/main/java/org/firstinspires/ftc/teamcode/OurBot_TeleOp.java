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
import java.lang.Math;

/*
 * This is our TeleOp.
 */

@TeleOp(name="OurBot: TeleOp", group="OurBot")
public class OurBot_TeleOp extends OpMode {

    /* Declare OpMode members. */
    OurBot_Hardware robot         = new OurBot_Hardware(); // Use the class created to define OurBot's hardware
    final double    CLAW_SPEED    = 0.005 ;                // Sets rate to move servo
    double          CLAW_OFFSET_1 = 0.0 ;                  // Offset from the servo's mid position
    double          CLAW_OFFSET_2 = 0.0 ;                  // Offset from the servo's mid position

    double left = 0.0;
    double right = 0.0;

    double jewelPosition = 37;
    boolean togglePressed = false;
    boolean frontAndBackSwitched = false;

    //double inertia = 0.15;                              // Not currently used

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
        robot.jewelArm.setPower(1.0);
        robot.jewelArm.setTargetPosition((int)jewelPosition);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.start) {
            togglePressed = true;
        }
        else if (togglePressed) {
            frontAndBackSwitched = !frontAndBackSwitched; // Inverse "frontAndBackSwitched"
            togglePressed = false;
        }

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
//        if (gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1
//                || gamepad1.right_stick_y > 0.1 || gamepad1.right_stick_y < -0.1) {
//            inertia += 0.01;
//            inertia = Range.clip(inertia, -1.0, 1.0);
//            if (!frontAndBackSwitched) {
//                left = -gamepad1.left_stick_y * inertia;
//                right = -gamepad1.right_stick_y * inertia;
//                left = -(gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y));
//                right = -(gamepad1.right_stick_y*Math.abs(gamepad1.right_stick_y));
//            } else { // Drive as if the back is the front
//                left = gamepad1.right_stick_y * inertia;
//                right = gamepad1.left_stick_y * inertia;
//                left = gamepad1.right_stick_y*Math.abs(gamepad1.right_stick_y);
//                right = gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y);
//            }
//        }
//        else {
//            left = 0; right = 0;
//            inertia = 0.15;
//        }
        if (!frontAndBackSwitched) {
            left = -(gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y));
            right = -(gamepad1.right_stick_y*Math.abs(gamepad1.right_stick_y));
        } else { // Drive as if the back is the front
            left = gamepad1.right_stick_y*Math.abs(gamepad1.right_stick_y);
            right = gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y);
        }

        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper || gamepad2.right_bumper)
            CLAW_OFFSET_1 += CLAW_SPEED;
        else if (gamepad1.left_bumper || gamepad2.left_bumper)
            CLAW_OFFSET_1 -= CLAW_SPEED;

        if (gamepad2.right_trigger > 0.1)
            CLAW_OFFSET_2 += CLAW_SPEED;
        else if (gamepad2.left_trigger > 0.1)
            CLAW_OFFSET_2 -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        CLAW_OFFSET_1 = Range.clip(CLAW_OFFSET_1, -0.15, 0.30);
        robot.dunkClawLeft1.setPosition(0.5 + CLAW_OFFSET_1);
        robot.dunkClawRight1.setPosition(0.5 - CLAW_OFFSET_1);
        CLAW_OFFSET_2 = Range.clip(CLAW_OFFSET_2, -0.15, 0.30);
        robot.dunkClawLeft2.setPosition(0.5 + CLAW_OFFSET_1);
        robot.dunkClawRight2.setPosition(0.5 - CLAW_OFFSET_1);

//        // Use gamepad buttons to move the slides up (Y) and down (A)
//        if (gamepad1.y || gamepad2.y) {
//            robot.leftSlide.setPower(0.6);
//            robot.rightSlide.setPower(0.6);
//        }
//        else if (gamepad1.a || gamepad2.a) {
//            robot.leftSlide.setPower(-0.5);
//            robot.rightSlide.setPower(-0.5);
//        }
//        else {
//            robot.leftSlide.setPower(0.0);
//            robot.rightSlide.setPower(0.0);
//        }
        robot.leftSlide.setPower(-gamepad2.right_stick_y);
        robot.rightSlide.setPower(-gamepad2.right_stick_y);

//        if (gamepad1.right_trigger > 0.25 || gamepad2.right_trigger > 0.25) {
//            robot.slideClaw.setPower(0.3);
//        }
//        else if (gamepad1.left_trigger > 0.25 || gamepad2.left_trigger > 0.25) {
//            robot.slideClaw.setPower(-0.3);
//        }
//        else {
//            robot.slideClaw.setPower(0);
//        }
        robot.slideClaw.setPower(-gamepad2.right_stick_x);

        // Use gamepad buttons to move the dunk claw up (DPAD_UP) and down (DPAD_DOWN)
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            robot.dunkClawArm.setPower(0.5);
        }
        else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            robot.dunkClawArm.setPower(-0.5);
        }
        else {
            robot.dunkClawArm.setPower(0);
        }

        // Move the jewel arm so it doesn't get in the way
        if (gamepad1.b || gamepad2.b)
            jewelPosition++;
        else if (gamepad1.x || gamepad2.x)
            jewelPosition--;

        robot.jewelArm.setTargetPosition((int)jewelPosition);

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", CLAW_OFFSET_1);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        //telemetry.addData("inertia", "%.2f", inertia);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
