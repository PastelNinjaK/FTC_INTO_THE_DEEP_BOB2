package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp2425M")
public class TeleOp2425M extends LinearOpMode {
  private DcMotor LFDrive;
  private DcMotor RFDrive;
  private DcMotor LRDrive;
  private DcMotor RRDrive;
  private DcMotor SlideMotor;
  private Servo TiltServoR;
  private Servo TiltServoL;
  private Servo IntakeServo;

  private double SlidePosition = 0;
  private double leftOutput = 0;
  private double rightOutput = 0;
  private static final double ARM_TICKS_PER_DEGREE = 0.404;

  // Define minimum and maximum slide positions
  private static final int SLIDE_MIN_POSITION = 0; // Lowest allowed position
  private static final int SLIDE_MAX_POSITION = 1000; // Highest allowed position

  @Override
  public void runOpMode() {
    // Initialize motors
    LFDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
    RFDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
    LRDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
    RRDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
    SlideMotor = hardwareMap.get(DcMotor.class, "slide_motor");
    TiltServoR = hardwareMap.get(Servo.class, "right_tilt_servo");
    TiltServoL = hardwareMap.get(Servo.class, "left_tilt_servo");
    IntakeServo = hardwareMap.get(Servo.class, "intake_servo");

    // Initialize servo positions
    TiltServoR.setPosition(0.0);
    TiltServoL.setPosition(0.0195);
    IntakeServo.setPosition(0);

    // Set motor brake behavior
    LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Reverse necessary motor directions
    LRDrive.setDirection(DcMotor.Direction.REVERSE);
    RFDrive.setDirection(DcMotor.Direction.REVERSE);
    TiltServoR.setDirection(Servo.Direction.REVERSE);

    waitForStart();

    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // Drive control
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x1 = gamepad1.left_stick_x; // Strafe
        double x2 = gamepad1.right_stick_x; // Turn
        double leftFactor = gamepad1.left_trigger;
        double rightFactor = gamepad1.right_trigger;

        double LFPower = y - x2 - x1;
        double RFPower = y + x2 + x1;
        double LRPower = y - x2 + x1;
        double RRPower = y + x2 - x1;

        LFDrive.setPower(LFPower);
        RFDrive.setPower(RFPower);
        LRDrive.setPower(LRPower);
        RRDrive.setPower(RRPower);

        // Slide motor control with triggers
        if (gamepad1.left_trigger > 0) {
          leftOutput += 360 * leftFactor; // Decrease position
        }
        if (gamepad1.right_trigger > 0) {
          rightOutput += 360 * rightFactor; // Increase position
        }

        // Calculate the new target position
        SlidePosition = (leftOutput - rightOutput) * ARM_TICKS_PER_DEGREE;

        // Ensure SlidePosition is within bounds
        SlidePosition = Math.max(SLIDE_MIN_POSITION, Math.min(SlidePosition, SLIDE_MAX_POSITION));

        // Update slide motor position if necessary
        if (SlideMotor.getTargetPosition() != (int) SlidePosition) {
          SlideMotor.setTargetPosition((int) SlidePosition);
          ((DcMotorEx) SlideMotor).setVelocity(2100);
          SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Tilt servo control
        if (gamepad1.dpad_up) {
          TiltServoR.setPosition(0.0); // Neutral position
          TiltServoL.setPosition(0.0195);
        } else if (gamepad1.dpad_down) {
          TiltServoR.setPosition(0.35); // Tilted position
          TiltServoL.setPosition(0.375);
        }
      }
    }
  }
}
