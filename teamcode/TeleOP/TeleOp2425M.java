package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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


//  private static final double ARM_TICKS_PER_DEGREE = 0.404;
  final double low_rung = 578.7116112967108;
  final double high_rung = 1070.7473672576434;
  final double low_basket = 762.9220420998186;
  final double high_basket = 1296.1627628456572;
  final double low_chamber = 362.9915015404398;
  final double high_chamber = 762.9220420998186;
  final double slide_off = 10;




  // Define minimum and maximum slide positions
 

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

    IntakeServo.setPosition(0);

    // Set motor brake behavior
    LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Reverse necessary motor directions
    SlideMotor.setDirection(DcMotor.Direction.REVERSE);
    LRDrive.setDirection(DcMotor.Direction.REVERSE);
    RFDrive.setDirection(DcMotor.Direction.REVERSE);
    TiltServoR.setDirection(Servo.Direction.REVERSE);
    // Slide encoder
    SlideMotor.setTargetPosition(10);
    SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    waitForStart();

    if (opModeIsActive()) {
      TiltServoR.setPosition(0.0);
      TiltServoL.setPosition(0.0195);
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
        if (gamepad1.a) {
          slidePosition = low_chamber; // Decrease position
        }
        if (gamepad1.b){
          slidePosition = slide_off; // Decrease position
        }

        if(gamepad1.x){
          slidePosition = high_chamber;
        }
        if (gamepad1.y){
          slidePosition = low_rung;
        }

        if(gamepad1.left_bumper){
          slidePosition = low_basket;
        }
        
        if (gamepad1.right_bumper){
          slidePosition = high_basket;
        }
        
        if (gamepad1.right_trigger){
          slidePosition = high_rung;
        }
       
        // Update slide motor position if necessary
        SlideMotor.setTargetPosition((int) slidePosition);
        ((DcMotorEx) SlideMotor).setVelocity(2100); // Adjust velocity as needed


        // Tilt servo control
        if (gamepad1.dpad_up) {
          TiltServoR.setPosition(0.0); // Neutral position
          TiltServoL.setPosition(0.0195);
        } else if (gamepad1.dpad_down) {
          TiltServoR.setPosition(0.35); // Tilted position
          TiltServoL.setPosition(0.375);
        }// end of if

        //Intake Servo Control


        if(gamepad1.dpad_right){
          //Servo is Open
          IntakeServo.setPosition(0);
        }// end of if
        if(gamepad1.dpad_left){
          //Servo is closed
          IntakeServo.setPosition(0.5);
        }// end of if
      }
    }
  }
}
