/* Copyright (c) 2017 Brain Stormz, FIRST. All rights reserved.
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
 * Neither the name of Brain Stormz, nor FIRST, nor the names of its contributors may be used to
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import ftclib.*;
import hallib.*;

@Autonomous(name="OurM0Bot: Autonomous", group ="OurM0Bot")
public class OurM0Bot_Autonomous extends LinearOpMode implements FtcMenu.MenuButtons {
    public enum Alliance {
        ALLIANCE_RED,
        ALLIANCE_BLUE
    }
    public enum StartPosition {
        STARTPOSITION1,
        STARTPOSITION2
    }
    public enum EndPosition {
        ENDBOARD,
        ENDNONE
    }
    public enum RunMode {
        RUNMODE_AUTO,
        RUNMODE_DEBUG
    }

    // Menu option variables
    RunMode        runmode       = RunMode.RUNMODE_AUTO;
    Alliance       alliance      = Alliance.ALLIANCE_RED;
    int            delay         = 0;
    StartPosition  startposition = StartPosition.STARTPOSITION1;
    EndPosition    endposition   = EndPosition.ENDBOARD;

    /* Declare OpMode members. */
    private HalDashboard dashboard;
    OurM0Bot_Hardware robot         = new OurM0Bot_Hardware();
    ColorSensor       color_sensor;

    static final double     COUNTS_PER_MOTOR_REV      = 2240;    // Rev HD Hex v2 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION      = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES     = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1416);

    /**
     * Define the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    /**
     * Color sensor declaration
     */

    @Override public void runOpMode() {
        // Initialize the hardware -----------------------------------------------------------------
        robot.init(hardwareMap);
        color_sensor = hardwareMap.colorSensor.get("jewel");

        // Initialize dashboard --------------------------------------------------------------------
        dashboard = HalDashboard.createInstance(telemetry);

        // Initialize Vuforia ----------------------------------------------------------------------
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor
         * (on the RC phone); If no camera monitor is desired, use the parameterless
         * constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                                                    "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Or, don't activate the Camera Monitor view, to save power
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Our key
        parameters.vuforiaLicenseKey = "AfXF2Nr/////AAAAGdMsV8BsHUFiomBNGmOw3NEPpvX+GToNkTKFRAohdsZ57x+R4vVXCAhd45zcQxYy3psYjXYJaAQtxU9PkBtn2l3T8BTkpEq46FZGRakXKnidi6F3kOYYco4wemaLAii9TcfZ3T7U7BJ3fXs+wRecumf1+e406AOvdiv5EU6PhWsPqjWu25y2Ta4D13WjxTgW6klkpgsTMeeA/4bu1DhcUPkhkA/c+MIf1qFXzidUyuI0DVHH5eO9KxjFzGzAB1e/oQV6ePyPbpnU55xhVtY8vG0ruO8skOweWBP4W+ABjLBrDR+XXeMirX+MsnbbreP4ZDE0NUokTOMqEGS5o0JL/E7FrcVDIFMrr41hlFg8WUR/";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /*
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one
         * template, but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        doMenus();
        dashboard.displayPrintf(0, "Status: Ready to start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        // Pause the program for the selected delay period
        sleep(delay);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("VuMark", "%s visible", vuMark);
        telemetry.update();
        //sleep(10000);

        if (color_sensor.red()>color_sensor.blue()) {
            robot.leftDrive.setPower(.3);
            robot.rightDrive.setPower(-.3);
            sleep(250);
            robot.leftDrive.setPower(-.3);
            robot.rightDrive.setPower(.3);
            sleep(250);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
        }
        else {
            robot.leftDrive.setPower(-.3);
            robot.rightDrive.setPower(.3);
            sleep(250);
            robot.leftDrive.setPower(.3);
            robot.rightDrive.setPower(-.3);
            sleep(250);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
        }
    }

    /**
     * FUNCTIONS -----------------------------------------------------------------------------------
     */

    boolean DoTask (String taskname, RunMode debug)
    {
        dashboard.displayPrintf(0, taskname);
        if (debug == RunMode.RUNMODE_DEBUG)
        {
            dashboard.displayPrintf(1, "Press A to run, B to skip");
            while(opModeIsActive()) {
                if(gamepad1.a) {
                    dashboard.displayPrintf(1, "Run");
                    return true;
                }
                if(gamepad1.b) {
                    dashboard.displayPrintf(1, "Skip");
                    sleep(1000);
                    return false;
                }
            }
        }
        else
            return true;
        return true;
    }

    void DriveRobotTime(int time, double power)
    {
        DrivePowerAll(power);

        sleep(time);

        DrivePowerAll(0);
    }

    /**
     * DrivePowerAll drives the robot the specified number of inches at the specified power level.
     * @param inches How far to drive, can be negative
     * @param power Power level to set motors to
     */
    void DriveRobotPosition(double power, int inches)
    {
        double position = inches*COUNTS_PER_INCH;

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DrivePowerAll(power);

        robot.leftDrive.setTargetPosition((int)position);
        robot.rightDrive.setTargetPosition((int)position);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()) {
            dashboard.displayPrintf(3,"Left encoder: %d", robot.leftDrive.getCurrentPosition());
            dashboard.displayPrintf(4,"Right encoder: %d", robot.rightDrive.getCurrentPosition());
        }

        DrivePowerAll(0);

    }

    // FIXME: position equation
    void DriveRobotTurn (double power, int degree)
    {
        double position = degree*19;

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);

        robot.leftDrive.setTargetPosition((int)position);
        robot.rightDrive.setTargetPosition(-(int)position);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()) {
            dashboard.displayPrintf(3,"Left encoder: %d", robot.leftDrive.getCurrentPosition());
            dashboard.displayPrintf(4,"Right encoder: %d", robot.rightDrive.getCurrentPosition());
        }

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

    // MENU ----------------------------------------------------------------------------------------
    @Override
    public boolean isMenuUpButton() {
        return gamepad1.dpad_up;
    }

    @Override
    public boolean isMenuDownButton() {
        return gamepad1.dpad_down;
    }

    @Override
    public boolean isMenuEnterButton() {
        return gamepad1.dpad_right;
    }

    @Override
    public boolean isMenuBackButton() {
        return gamepad1.dpad_left;
    }

    private void doMenus() {
        FtcChoiceMenu modeMenu = new FtcChoiceMenu("Run Mode", null, this);
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance:", modeMenu, this);
        FtcChoiceMenu delayMenu = new FtcChoiceMenu("Delay:", allianceMenu, this);
        FtcChoiceMenu startpositionMenu = new FtcChoiceMenu("Start Position:", delayMenu, this);
        FtcChoiceMenu endpositionMenu = new FtcChoiceMenu("End Position:", startpositionMenu, this);

        modeMenu.addChoice("Auto", RunMode.RUNMODE_AUTO, true, allianceMenu);
        modeMenu.addChoice("Debug", RunMode.RUNMODE_DEBUG, false, allianceMenu);

        allianceMenu.addChoice("Red", Alliance.ALLIANCE_RED, true, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.ALLIANCE_BLUE, false, delayMenu);

        delayMenu.addChoice("0 seconds", 0, true, startpositionMenu);
        delayMenu.addChoice("5 seconds", 5000, false, startpositionMenu);
        delayMenu.addChoice("10 seconds", 10000, false, startpositionMenu);
        delayMenu.addChoice("15 seconds", 15000, false, startpositionMenu);

        startpositionMenu.addChoice("1", StartPosition.STARTPOSITION1, true, endpositionMenu);
        startpositionMenu.addChoice("2", StartPosition.STARTPOSITION2, false, endpositionMenu);

        endpositionMenu.addChoice("Balancing Stone", EndPosition.ENDBOARD, false, null);
        endpositionMenu.addChoice("None", EndPosition.ENDNONE, true, null);


        FtcMenu.walkMenuTree(modeMenu, this);
        runmode = (RunMode) modeMenu.getCurrentChoiceObject();
        alliance = (Alliance) allianceMenu.getCurrentChoiceObject();
        delay = (int) delayMenu.getCurrentChoiceObject();
        startposition = (StartPosition) startpositionMenu.getCurrentChoiceObject();
        endposition = (EndPosition) endpositionMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(9, "Mode: %s (%s)", modeMenu.getCurrentChoiceText(), runmode.toString());
        dashboard.displayPrintf(10, "Alliance: %s (%s)", allianceMenu.getCurrentChoiceText(), alliance.toString());
        dashboard.displayPrintf(11, "Delay = %d msec", delay);
        dashboard.displayPrintf(12, "Start position: %s (%s)", startpositionMenu.getCurrentChoiceText(), startposition.toString());
        dashboard.displayPrintf(13, "End Position = %s (%s)", endpositionMenu.getCurrentChoiceText(), endposition.toString());
    }
    // END MENU ------------------------------------------------------------------------------------
}
