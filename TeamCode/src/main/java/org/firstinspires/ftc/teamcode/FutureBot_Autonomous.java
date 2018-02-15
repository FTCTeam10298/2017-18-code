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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

@Autonomous(name="FutureBot: Autonomous", group ="FutureBot")
public class FutureBot_Autonomous extends LinearOpMode implements FtcMenu.MenuButtons {
    public enum Alliance {
        ALLIANCE_RED,
        ALLIANCE_BLUE
    }
    public enum StartPosition {
        STARTPOSITION1,
        STARTPOSITION2
    }
    public enum RunMode {
        RUNMODE_AUTO,
        RUNMODE_DEBUG
    }
    public enum Column {
        COLUMN_LEFT,
        COLUMN_CENTER,
        COLUMN_RIGHT
    }
    public enum GetExtraGlyph {
        NEVER,
        IF_NEAR,
        IF_NEAR_OR_CENTER,
        ALWAYS
    }

    // Menu option variables
    RunMode        runmode       = RunMode.RUNMODE_AUTO;
    Alliance       alliance      = Alliance.ALLIANCE_RED;
    int            delay         = 0;
    StartPosition  startposition = StartPosition.STARTPOSITION1;
    GetExtraGlyph getExtraGlyph = GetExtraGlyph.NEVER;

    /* Declare OpMode members. */
    private HalDashboard dashboard;
    FutureBot_Hardware   robot   = new FutureBot_Hardware();
    ColorSensor          color_sensor;
    ColorSensor          color_left;
    ColorSensor          color_right;

    static final double  COUNTS_PER_MOTOR_REV      = 1120;    // Rev HD Hex v2.1 Motor encoder
    static final double  DRIVE_GEAR_REDUCTION      = 1;       // This is < 1.0 if geared for torque
    static final double  WHEEL_DIAMETER_INCHES     = 4.0;     // For figuring circumference
    static final double  ERROR                     = .825;    //Constant for error
    static final double  COUNTS_PER_INCH           = (ERROR * COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1416);

    Column column = Column.COLUMN_CENTER;

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
        color_left = hardwareMap.colorSensor.get("left_square");
        color_right = hardwareMap.colorSensor.get("right_square");

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

        // Run though the menu ---------------------------------------------------------------------
        doMenus();
        dashboard.displayPrintf(0, "Status: Ready to start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        // Pause the program for the selected delay period
        sleep(delay);

        if (DoTask("Read VuMark", runmode)) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                column = Column.COLUMN_LEFT;
                dashboard.displayPrintf(6, "VuMark: LEFT visible");
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                column = Column.COLUMN_CENTER;
                dashboard.displayPrintf(6, "VuMark: CENTER visible");
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                column = Column.COLUMN_RIGHT;
                dashboard.displayPrintf(6, "VuMark: RIGHT visible");
            } else {
                column = Column.COLUMN_CENTER;
                dashboard.displayPrintf(6, "VuMark: UNKNOWN visible");
            }
        }

        if (DoTask("Init", runmode)) {
            // Init - optimized

            //robot.jewelHitter.setPosition(0);
            //sleep(100);
            robot.jewelArm.setPosition(0);
            sleep(750);
        }

        if (DoTask("Knock off jewel", runmode)) {
            dashboard.displayPrintf(14, "" + color_sensor.red());
            dashboard.displayPrintf(15, "" + color_sensor.blue());
            if ((color_sensor.red() > color_sensor.blue() && alliance == Alliance.ALLIANCE_RED)
                    || (color_sensor.blue() > color_sensor.red() && alliance == Alliance.ALLIANCE_BLUE)) {
                robot.jewelHitter.setPosition(0);
                sleep(500);
                robot.jewelArm.setPosition(1);
                sleep(500);
            } else if ((color_sensor.red() > color_sensor.blue() && alliance == Alliance.ALLIANCE_BLUE)
                    || (color_sensor.blue() > color_sensor.red() && alliance == Alliance.ALLIANCE_RED)) {
                robot.jewelHitter.setPosition(1);
                sleep(500);
                robot.jewelArm.setPosition(1);
                sleep(500);
            } else {
                robot.jewelArm.setPosition(1);
                sleep(900);
            }
        } else {
            robot.jewelArm.setPosition(1);
            sleep(900);
        }

        // drive
        if (DoTask("Drive to CryptoBox", runmode)) {
            if (startposition == StartPosition.STARTPOSITION1) {
                if (alliance == Alliance.ALLIANCE_RED) {
                    if (column == Column.COLUMN_RIGHT) {
                        DriveRobotPosition(0.2, -29);
                    } else if (column == Column.COLUMN_CENTER) {
                        DriveRobotPosition(0.2, -36);
                    } else {
                        DriveRobotPosition(0.2, -43);
                    }
                } else { // Alliance.ALLIANCE_BLUE
                    if (column == Column.COLUMN_LEFT) {
                        DriveRobotPosition(0.2, 29);
                    } else if (column == Column.COLUMN_CENTER) {
                        DriveRobotPosition(0.2, 36);
                    } else {
                        DriveRobotPosition(0.2, 43);
                    }
                }

                sleep(500);
                DriveRobotTurn(0.25, -90);
                sleep(500);
                robot.IntakeLeft.setPower(.6);
                robot.IntakeRight.setPower(.4);
                sleep(1000);
                robot.IntakeLeft.setPower(.5);
                robot.IntakeRight.setPower(.5);
                DriveRobotPosition(0.25, 6);
            }
            else { // StartPosition.STARTPOSITION2 -----------------------------------------------
                if (alliance == Alliance.ALLIANCE_RED) {
                    DriveRobotPosition(.2, -28);
                    sleep(500);
                    if (column == Column.COLUMN_CENTER) {
                        DriveRobotTurn(.25, 130);
                        sleep(500);
                        DriveRobotPosition(.1, 8);
                        sleep(500);
                    }
                    else if (column == Column.COLUMN_LEFT){
                        DriveRobotTurn(.25, 100);
                        sleep(500);
                        DriveRobotPosition(.1, 14);
                        sleep(500);
                        DriveRobotTurn(.25,25);
                    }
                    else {
                        DriveRobotTurn(.25, 160);
                        sleep(500);
                        DriveRobotPosition(.1, 7);
                        sleep(500);
                    }

                } else { // Alliance.ALLIANCE_BLUE
                    DriveRobotPosition(.1, -26);
                    if (column == Column.COLUMN_CENTER) {
                        DriveRobotPosition(.1,-4);
                        sleep(500);
                        DriveRobotTurn(.25, -147);
                        sleep(500);
                        DriveRobotPosition(.1, 8);
                        sleep(500);
                    }
                    else if (column == Column.COLUMN_LEFT){
                        DriveRobotPosition(.1,-4);
                        sleep (500);
                        DriveRobotTurn(.25, -165);
                        sleep(500);
                        DriveRobotPosition(.1, 7);
                        sleep(500);
                    }
                    else { // Column.COLUMN_RIGHT
                        sleep(500);
                        DriveRobotTurn(.25, -137);
                        sleep(500);
                        DriveRobotPosition(.1, 14);
                        sleep(500);
                    }
                }
                robot.IntakeLeft.setPower(.6);
                robot.IntakeRight.setPower(.4);
                sleep(500);
                robot.IntakeLeft.setPower(.5);
                robot.IntakeRight.setPower(.5);
            }
        }

        // Drop glyph
        DriveRobotPosition(0.25, -5);

        if (startposition == StartPosition.STARTPOSITION1 &&
                (getExtraGlyph == GetExtraGlyph.ALWAYS ||
                (getExtraGlyph == GetExtraGlyph.IF_NEAR_OR_CENTER && column == Column.COLUMN_CENTER) ||
                (
                    (getExtraGlyph == GetExtraGlyph.IF_NEAR_OR_CENTER || getExtraGlyph == GetExtraGlyph.IF_NEAR) &&
                    (
                        (alliance == Alliance.ALLIANCE_RED && column == Column.COLUMN_RIGHT) ||
                        (alliance == Alliance.ALLIANCE_BLUE && column == Column.COLUMN_LEFT)
                    )
                )
            )
        ) {
            DrivePushGlyph(1, 0.35);

            DriveRobotTurn(.5, 180);

            robot.dunkClawArm.setPower(1);
            sleep(300);
            robot.dunkClawArm.setPower(0);

            robot.spinnyClaw.setPosition(1);
            robot.dunkClaw1.setPosition(0);
            robot.dunkClaw2.setPosition(0);

            sleep(500);

            robot.dunkClawArm.setPower(-.1);
            sleep(500);
            robot.dunkClawArm.setPower(0);

            DriveRobotPosition(1, 42);
            robot.dunkClaw1.setPosition(1);
            robot.dunkClaw2.setPosition(1);
            sleep(1000);
            DriveRobotPosition(1, -10);
            robot.dunkClawArm.setPower(1);
            sleep(250);
            DriveRobotPosition(1, -29);
            robot.dunkClaw1.setPosition(0.3);
            robot.dunkClaw2.setPosition(0.3);
            robot.dunkClawArm.setPower(0);
            sleep(500);
            DriveRobotPosition(0.5, 5);
            DrivePushGlyph(1, 1.0);
        }
        else { // No extra glyphs
            DrivePushGlyph(1, 0.35);
            if (startposition==StartPosition.STARTPOSITION2){
                if (alliance==Alliance.ALLIANCE_RED)
                    DriveRobotTurn(.3, -90);
                else
                    DriveRobotTurn(.3, 90);
            }
            else {
                DriveRobotTurn(.3, 180);
                DriveRobotPosition(1, 42);
                DriveRobotTurn(1, 360);
                DriveRobotPosition(1, -10);
            }
        }

    }

    /*
     * FUNCTIONS -----------------------------------------------------------------------------------
     */

    boolean DoTask (String taskname, RunMode debug)
    {
        dashboard.displayPrintf(0, taskname);
        if (debug == RunMode.RUNMODE_DEBUG)
        {
            dashboard.displayPrintf(1, "Press A to run, B to skip");
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    dashboard.displayPrintf(1, "Run");
                    return true;
                }
                if (gamepad1.b) {
                    dashboard.displayPrintf(1, "Skip");
                    sleep(1000);
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * DriveRobotTime drives the robot the set number of inches at the given power level.
     * @param ms How long to drive
     * @param power Power level to set motors to, negative will drive the robot backwards
     */
    void DriveRobotTime(int ms, double power)
    {
        DrivePowerAll(power);
        sleep(ms);
        DrivePowerAll(0);
    }

    /**
     * DriveRobotPosition drives the robot the set number of inches at the given power level.
     * @param inches How far to drive, can be negative
     * @param power Power level to set motors to
     */
    void DriveRobotPosition(double power, double inches)
    {
        double position = inches*COUNTS_PER_INCH;

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DrivePowerAll(power);

        robot.frontLeftDrive.setTargetPosition((int)position);
        robot.frontRightDrive.setTargetPosition((int)position);
        robot.backLeftDrive.setTargetPosition((int)position);
        robot.backRightDrive.setTargetPosition((int)position);

        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
                                       // as isBusy can be a bit unreliable
            while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy()
                    && robot.backRightDrive.isBusy()) {
                dashboard.displayPrintf(3, "Left front encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(4, "Right front encoder: %d", robot.frontRightDrive.getCurrentPosition());
                dashboard.displayPrintf(5, "Left back encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(6, "Right back encoder %d", robot.backRightDrive.getCurrentPosition());
            }
            sleep(10);
        }

        DrivePowerAll(0);
    }

    /**
     * DriveRobotSquare drives the robot forward at the set power level up to a line
     * @param power Power level to drive forward with
     */
    void DriveRobotSquare (double power)
    {
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean done = false;

        while (!done) {
            dashboard.displayPrintf(3, "Color right red: " + color_right.red());
            dashboard.displayPrintf(4, "Color right blue: " + color_right.blue());
            dashboard.displayPrintf(5, "Color left red: " + color_left.red());
            dashboard.displayPrintf(6, "Color left blue: " + color_left.blue());
            if (alliance == Alliance.ALLIANCE_RED) {
                if (color_left.red()<15)
                    robot.frontLeftDrive.setPower(power);
                else
                    robot.frontLeftDrive.setPower(0);
                if (color_right.red()<15)
                    robot.frontRightDrive.setPower(power);
                else
                    robot.frontRightDrive.setPower(0);
                if ((color_left.red()>15)&&(color_right.red()>15))
                    done = true;
            } else { // Alliance.ALLIANCE_BLUE
                if (color_left.blue()<15)
                    robot.frontLeftDrive.setPower(power);
                else
                    robot.frontLeftDrive.setPower(-power/5);
                if (color_right.blue()<15)
                    robot.frontLeftDrive.setPower(power);
                else
                    robot.frontLeftDrive.setPower(-power/5);
                if ((color_left.blue()>15)&&(color_right.blue()>15))
                    done = true;
            }
        }
        DrivePowerAll(0);
        dashboard.displayPrintf(3, "Color right red: " + color_right.red());
        dashboard.displayPrintf(4, "Color right blue: " + color_right.blue());
        dashboard.displayPrintf(5, "Color left red: " + color_left.red());
        dashboard.displayPrintf(6, "Color left blue: " + color_left.blue());
        sleep(500);
    }

    void DriveRobotTurn (double power, double degree)
    {
//        double position = degree*DRIVE_GEAR_REDUCTION*15.2; // FIXME: Magic number
        double position = degree*COUNTS_PER_INCH*0.20672; // FIXME: Magic number

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setPower(power);
        robot.frontRightDrive.setPower(-power);
        robot.backLeftDrive.setPower(power);
        robot.backRightDrive.setPower(-power);

        robot.frontLeftDrive.setTargetPosition((int)position);
        robot.frontRightDrive.setTargetPosition(-(int)position);
        robot.backLeftDrive.setTargetPosition((int)position);
        robot.backRightDrive.setTargetPosition(-(int)position);

        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
                                       // as isBusy can be a bit unreliable
            while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy()
                    && robot.backRightDrive.isBusy()) {
                dashboard.displayPrintf(3, "Left front encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(4, "Right front encoder: %d", robot.frontRightDrive.getCurrentPosition());
                dashboard.displayPrintf(5, "Left back encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(6, "Right back encoder %d", robot.backRightDrive.getCurrentPosition());
            }
            sleep(10);
        }

        DrivePowerAll(0);
    }

    /**
     * DrivePowerAll sets all of the drive train motors to the specified power level.
     * @param power Power level to set motors to
     */
    void DrivePowerAll (double power)
    {
        robot.frontLeftDrive.setPower(power);
        robot.frontRightDrive.setPower(power);
        robot.backLeftDrive.setPower(power);
        robot.backRightDrive.setPower(power);
    }

    void DrivePushGlyph (int pushes, double power)
    {
        for (int i = 0; i < pushes; i++) {
            DriveRobotPosition(power, 7);
            DriveRobotPosition(power, -7);
        }
    }

    // MENU ----------------------------------------------------------------------------------------
    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up; }

    @Override
    public boolean isMenuDownButton() { return gamepad1.dpad_down; }

    @Override
    public boolean isMenuEnterButton() { return gamepad1.dpad_right; }

    @Override
    public boolean isMenuBackButton() { return gamepad1.dpad_left; }

    private void doMenus() {
        FtcChoiceMenu<RunMode> modeMenu = new FtcChoiceMenu<>("Run Mode", null, this);
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", modeMenu, this);
        FtcValueMenu delayMenu = new FtcValueMenu("Delay:", allianceMenu, this, 0, 20000, 1000, 0, "%.0f msec");
        FtcChoiceMenu<StartPosition> startPositionMenu = new FtcChoiceMenu<>("Start Position:", delayMenu, this);
        FtcChoiceMenu<GetExtraGlyph> extraGlyphMenu = new FtcChoiceMenu<>("Get extra glyph:", startPositionMenu, this);

        modeMenu.addChoice("Auto", RunMode.RUNMODE_AUTO, true, allianceMenu);
        modeMenu.addChoice("Debug", RunMode.RUNMODE_DEBUG, false, allianceMenu);

        allianceMenu.addChoice("Red", Alliance.ALLIANCE_RED, true, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.ALLIANCE_BLUE, false, delayMenu);

        delayMenu.setChildMenu(startPositionMenu);

        startPositionMenu.addChoice("1", StartPosition.STARTPOSITION1, true, extraGlyphMenu);
        startPositionMenu.addChoice("2", StartPosition.STARTPOSITION2, false, extraGlyphMenu);

        extraGlyphMenu.addChoice("Never", GetExtraGlyph.NEVER, true, null);
        extraGlyphMenu.addChoice("If near", GetExtraGlyph.IF_NEAR, true, null);
        extraGlyphMenu.addChoice("If near or center", GetExtraGlyph.IF_NEAR_OR_CENTER, true, null);
        extraGlyphMenu.addChoice("Always", GetExtraGlyph.ALWAYS, true, null);

        FtcMenu.walkMenuTree(modeMenu, this);
        runmode = modeMenu.getCurrentChoiceObject();
        alliance = allianceMenu.getCurrentChoiceObject();
        delay = (int) delayMenu.getCurrentValue();
        startposition = startPositionMenu.getCurrentChoiceObject();
        getExtraGlyph = extraGlyphMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(9, "Mode: %s (%s)", modeMenu.getCurrentChoiceText(), runmode.toString());
        dashboard.displayPrintf(10, "Alliance: %s (%s)", allianceMenu.getCurrentChoiceText(), alliance.toString());
        dashboard.displayPrintf(11, "Delay = %d msec", delay);
        dashboard.displayPrintf(12, "Start position: %s (%s)", startPositionMenu.getCurrentChoiceText(), startposition.toString());
        dashboard.displayPrintf(13, "Get extra glyph: %s (%s)", extraGlyphMenu.getCurrentChoiceText(), getExtraGlyph.toString());
    }
    // END MENU ------------------------------------------------------------------------------------
}
