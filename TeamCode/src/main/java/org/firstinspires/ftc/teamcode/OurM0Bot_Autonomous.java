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
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="OurM0Bot: Autonomous", group ="OurM0Bot")
public class OurM0Bot_Autonomous extends LinearOpMode {

    OurM0Bot_Hardware robot        = new OurM0Bot_Hardware(); // use the class created to define OurM0Bot's hardware

    //public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    ColorSensor color_sensor;

    /**
     * Color sensor declaration
     */

    @Override public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor
         * (on the RC phone); If no camera monitor is desired, use the parameterless
         * constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                                                    "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
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

        color_sensor = hardwareMap.colorSensor.get("jewel");
        telemetry.update();

        relicTrackables.activate();

        /* ******************* */
        waitForStart();
        /* ******************* */

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

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
