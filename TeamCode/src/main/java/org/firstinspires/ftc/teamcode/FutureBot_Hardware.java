package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */

public class FutureBot_Hardware
{
    /* Public OpMode members. */
    public DcMotor frontLeftDrive     = null;
    public DcMotor backLeftDrive      = null;
    public DcMotor frontRightDrive    = null;
    public DcMotor backRightDrive     = null;
    public DcMotor dunkClawArm        = null;
    public DcMotor relicOut           = null;
    public DcMotor IntakeLeft         = null;
    public DcMotor IntakeRight        = null;
    public CRServo relicLift          = null;
    public Servo   dunkClaw1          = null;
    public Servo   dunkClaw2          = null;
    public Servo   spinnyClaw         = null;
    public Servo   relicElbow         = null;
    public Servo   relicClaw          = null;
    public Servo   jewelArm           = null;
    public Servo   jewelHitter        = null;
    public Servo   intakeRotateRight  = null;
    public Servo   intakeRotateLeft   = null;

    /* Local OpMode members. */
    HardwareMap    hwMap              = null;

    /* Constructor */
    public FutureBot_Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        frontLeftDrive = hwMap.dcMotor.get("front_left_drive");
        backLeftDrive = hwMap.dcMotor.get("back_left_drive");
        frontRightDrive = hwMap.dcMotor.get("front_right_drive");
        backRightDrive = hwMap.dcMotor.get("back_right_drive");

        dunkClawArm = hwMap.dcMotor.get("back_arm");
        relicOut    = hwMap.dcMotor.get("relicOut");

        IntakeLeft = hwMap.dcMotor.get("intakeLeft");
        IntakeRight = hwMap.dcMotor.get("intakeRight");

        // Set direction for all motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        dunkClawArm.setDirection(DcMotor.Direction.FORWARD);
        relicOut.setDirection(DcMotor.Direction.REVERSE);

        IntakeLeft.setDirection(DcMotor.Direction.FORWARD);
        IntakeRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        dunkClawArm.setPower(0);
        relicOut.setPower(0);

        IntakeLeft.setPower(0);
        IntakeRight.setPower(0);

        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set (almost) all motors to run with encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dunkClawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // These motors do not use encoders
        IntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize all installed servos
        dunkClaw1 = hwMap.servo.get("right_back_claw");
        dunkClaw2 = hwMap.servo.get("right_back_claw_2");

        spinnyClaw = hwMap.servo.get("dunk_claw_rotate");

        jewelArm = hwMap.servo.get("jewel_arm");
        jewelArm.setPosition(1);

        jewelHitter = hwMap.servo.get("jewelHitter");
        jewelHitter.setPosition(0.5);

        relicElbow = hwMap.servo.get("relicElbow");
//        relicElbow.setPosition(-1.0);

        relicClaw = hwMap.servo.get("relicClaw");
//        relicClaw.setPosition(0);

        relicLift = hwMap.crservo.get("relicLift");
        relicLift.setPower(0);
        relicLift.setDirection(CRServo.Direction.FORWARD);

        intakeRotateRight = hwMap.servo.get("intakeRotateRight");
        intakeRotateLeft = hwMap.servo.get("intakeRotateLeft");
    }
}

