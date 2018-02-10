package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */

public class FutureBot_Hardware
{
    /* Public OpMode members. */
    public DcMotor frontLeftMotor     = null;
    public DcMotor backLeftMotor      = null;
    public DcMotor frontRightMotor    = null;
    public DcMotor backRightMotor     = null;
    public DcMotor dunkClawArm        = null;
    public DcMotor relicOut           = null;
    public DcMotor leftIntake         = null;
    public DcMotor rightIntake        = null;
    public CRServo relicLift          = null;
    public Servo   dunkClawRight1     = null;
    public Servo   dunkClawRight2     = null;
    public Servo   spinnyClaw         = null;
    public Servo   relicElbow         = null;
    public Servo   relicClaw          = null;
    public Servo   jewelArm           = null;
    public Servo   jewelHitter        = null;
    public Servo   intakeRotateRight  = null;
    public Servo   intakeRotateLeft   = null;

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

        dunkClawArm = hwMap.get(DcMotor.class, "back_arm");
        relicOut    = hwMap.get(DcMotor.class, "relicOut");

        leftIntake  = hwMap.get(DcMotor.class, "intakeLeft");
        rightIntake = hwMap.get(DcMotor.class, "intakeRight");

        // Set direction for all motors
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        dunkClawArm.setDirection(DcMotor.Direction.FORWARD);
        relicOut.setDirection(DcMotor.Direction.REVERSE);

        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        dunkClawArm.setPower(0);
        relicOut.setPower(0);

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set (almost) all motors to run with encoders
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dunkClawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // These motors do not use encoders
        //relicOut.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize all installed servos
        dunkClawRight1 = hwMap.get(Servo.class, "right_back_claw");

        dunkClawRight2 = hwMap.get(Servo.class, "right_back_claw_2");

        spinnyClaw = hwMap.get(Servo.class, "dunk_claw_rotate");

        jewelArm = hwMap.get(Servo.class, "jewel_arm");
        jewelArm.setPosition(1);

        jewelHitter = hwMap.get(Servo.class,"jewelHitter");
        jewelHitter.setPosition(0.5);

        relicElbow = hwMap.get(Servo.class, "relicElbow");
        relicElbow.setPosition(-1.0);

        relicClaw = hwMap.get(Servo.class, "relicClaw");
        relicClaw.setPosition(0);

        relicLift = hwMap.get(CRServo.class, "relicLift");
        relicLift.setPower(0);
        relicLift.setDirection(CRServo.Direction.FORWARD);
    }
}

