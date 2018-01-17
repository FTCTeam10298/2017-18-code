package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */

public class FutureBot_Hardware
{
    /* Public OpMode members. */
    public DcMotor frontLeftMotor  = null;
    public DcMotor backLeftMotor   = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backRightMotor  = null;

    /* local OpMode members. */
    HardwareMap hwMap               = null;
    private ElapsedTime period      = new ElapsedTime();

    /* Constructor */
    public FutureBot_Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        frontLeftMotor = hwMap.dcMotor.get("front_left_drive");
        backLeftMotor = hwMap.dcMotor.get("back_left_drive");
        frontRightMotor = hwMap.dcMotor.get("front_right_drive");
        backRightMotor = hwMap.dcMotor.get("back_right_drive");

        // Set direction for all motors
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        // Set (almost) all motors to run with encoders.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // These motors does not use encoders

        // Define and initialize all installed servos.
    }
}

