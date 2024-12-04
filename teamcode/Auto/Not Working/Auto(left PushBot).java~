package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.AutoOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;



@Autonomous(name = "Auto2425(left pushbot)")
public class Auto2425 extends LinearOpMode {
  private DcMotor leftDrive;
  private DcMotor rightDrive;
  private DcMotor armMotor;
  private CRServo wheelServo; // Declare thAuto(left PushBot).javae servo
  private Servo wrist;

  //Java function for programing the motors
  public void program(double leftPower, double rightPower, double delay){
    leftDrive.setPower(leftPower);
    rightDrive.setPower(rightPower);
    sleep((long) 1000 * delay);
  }//end of function

  public double TimeDelay(double dist, doucle speed){
    return dist/speed;
    //DIST AND SPEED NEED TO BE IN METERS AND METERS PER SECOND RESPECTIVELY!
  }
  // TimeDelay will replace the thrid parameter for program
  // distance for the full arena is 12ft/365.76cm/ 3.657m (I hate the impreal system)
  // we need to calculate the max sped of our robot.

  final double ARM_TICKS_PER_DEGREE = 19.7924893140647; // Encoder ticks per degree of arm rotation
  final double ARM_LEVEL_ONE  = 190 * ARM_TICKS_PER_DEGREE;// This needs to be calibrated, I just guessed on this one.

  @Override
  public void runOpMode() {
    // Initialize motors
    leftDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
    rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
    armMotor = hardwareMap.get(DcMotor.class, "left_arm");
    wrist = hardwareMap.get(Servo.class, "wrist");

    // Initialize servo
    wheelServo = hardwareMap.get(CRServo.class, "intake");
    wrist.setPosition(0.45);

    // Reverse necessary motor directions
    rightDrive.setDirection(DcMotor.Direction.REVERSE);
    ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS); // Set current alert to 5 Amps
    armMotor.setTargetPosition(0); // Set initial target position to 0
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Configure for RunToPosition mode
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder



    ;
    waitForStart();

    if (opModeIsActive()) {
        //Scoring all 3 scoringe elements and lvl1 ascend
        //sleep time need calibration
        //Assuming we place our robot at F2 (open link from next line) and our robot has a maximum speed of 0.5m/s.
        // Image link: https://firstroboticsbc.org/ftc/ftc-team-resources/intothedeep-autonomous-programs/
        // image name: Figure 9.4 TILE placement
        // 1 tile = 2ft / 0.6096 m
        // 0.5 is NOT the max speed of our robot, it's just me guessing.
        // once we actually get the top speed of our robot, we use that value as seconnd parameter of the TimeDelay function.
        double time = TimeDelay(0.6096,0.5);// we only need 1 because this is measuring how long it takes for the robot to travel 1 tile.
        program(-1.0, -1.0, time); // Backwards (needs to be adjusted to acount for the support beam of the submerisible)
        program(-1.0, 1.0, 1);  // Rotate Counter clockwise(can't change this rn cuz we need to find the time it take for the robot to turn 90 degrees).
        program(1.0, 1.0, time * 3.5); // Forward (Time is multiplied by 3.5 because the variable time is calculating how long it take for the robot to travel across 1 Tile).

        program(-1.0, 1.0, 1); // Rotate Lining up to the 3 scoring elements(Counter Clockwise)(Same for the first rotate)
        program(1.0, 1.0, time * 2);// Forward 2 tile to lineup all with all the scoring elements
        program(1.0, -1.0, 1);// Rotate Clockwise to lineup all with all the scoring elements
        program(1.0, 1.0 , time); // Forward to lineup all with all the scoring elements
        program(1.0, -1.0, 1);// Rotate Clockwise to lineup all with all the scoring elements

        program(1.0, 1.0, time * 3.5);// Forward to push all the scoring elements to the observation zone

        // Stop motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //while (opModeIsActive()) was removed because it's like a big for loop. We only want this to run the code once.
    }
  }

// Code essential for arm
//  public DcMotor armMotor = null; // The arm motor declaration
//
//  // Constants for arm motor calculations and positions
//  final double ARM_TICKS_PER_DEGREE = 19.7924893140647; // Encoder ticks per degree of arm rotation
//  final double ARM_COLLAPSED_INTO_ROBOT  = 0;
//  final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
//  final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
//  final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
//  final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
//  final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
//  final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;
//  final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE; // Adjustment factor
//
//  // Variables for controlling arm position
//  double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
//  double armPositionFudgeFactor;
//
//  @Override
//  public void runOpMode() {
//    // Initialize the arm motor
//    armMotor = hardwareMap.get(DcMotor.class, "left_arm");
//
//    // Configure the arm motor
//    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Enable braking
//    ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS); // Set current alert to 5 Amps
//    armMotor.setTargetPosition(0); // Set initial target position to 0
//    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Configure for RunToPosition mode
//    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
//
//    // Wait for the game driver to press play
//    waitForStart();
//
//    // Main loop
//    while (opModeIsActive()) {
//      // Calculate the fudge factor based on triggers
//      armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger - gamepad1.left_trigger);
//
//      // Update arm position based on gamepad inputs
//      if (gamepad1.right_bumper) {
//        armPosition = ARM_COLLECT; // Collecting position
//      } else if (gamepad1.left_bumper) {
//        armPosition = ARM_CLEAR_BARRIER; // Clear barrier position
//      } else if (gamepad1.y) {
//        armPosition = ARM_SCORE_SAMPLE_IN_LOW; // Low scoring position
//      } else if (gamepad1.dpad_left) {
//        armPosition = ARM_COLLAPSED_INTO_ROBOT; // Collapsed position
//      } else if (gamepad1.dpad_right) {
//        armPosition = ARM_SCORE_SPECIMEN; // High scoring position
//      } else if (gamepad1.dpad_up) {
//        armPosition = ARM_ATTACH_HANGING_HOOK; // Hanging position
//      } else if (gamepad1.dpad_down) {
//        armPosition = ARM_WINCH_ROBOT; // Winch position
//      }
//
//      // Set the motor target position and velocity
//      armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
//      ((DcMotorEx) armMotor).setVelocity(2100); // Set motor velocity
//      armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Ensure the motor runs to the target position
//
//      // Telemetry for feedback
//      if (((DcMotorEx) armMotor).isOverCurrent()) {
//        telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
//      }
//      telemetry.addData("armTarget: ", armMotor.getTargetPosition());
//      telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
//      telemetry.update();
//    }
//  }
