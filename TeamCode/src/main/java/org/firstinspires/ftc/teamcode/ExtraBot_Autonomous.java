/* Copyright (c) 2018 Brain Stormz. All rights reserved.
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
 * Neither the name of Brain Stormz, nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="ExtraBot: Autonomous", group ="ExtraBot")
public class ExtraBot_Autonomous extends LinearOpMode {

    ExtraBot_Hardware robot = new ExtraBot_Hardware();

    @Override public void runOpMode() {

        robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */
        DriveRobotTime(1500, 1.0);
        robot.pivotClaw1.setPosition(0);
        robot.pivotClaw2.setPosition(1);
        DriveRobotTime(500, -1.0);
        DriveRobotTime(500, 1.0);
        DriveRobotTime(500, -1.0);
        DriveRobotTime(500, 1.0);
        DriveRobotTime(500, -0.6);
    }

    /*
     * FUNCTIONS -----------------------------------------------------------------------------------
     */

    /**
     * DriveRobotTime drives the robot the specified number of inches at the specified power level.
     * @param ms How long to drive
     * @param power Power level to set motors to, can be negative
     */
    void DriveRobotTime(int ms, double power)
    {
        DrivePowerAll(power);
        sleep(ms);
        DrivePowerAll(0);
    }

    /**
     * DrivePowerAll sets all of the drive train motors to the specified power level.
     * @param power Power level to set motors to
     */
    void DrivePowerAll (double power)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);
    }
}
