package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TeleOp2425M3")
public class TeleOp2425M3 extends LinearOpMode {
  private DcMotor LFDrive;
  private DcMotor RFDrive;
  private DcMotor LRDrive;
  private DcMotor RRDrive;
  private DcMotorEx SlideMotor;
  private Servo TiltServoR;
  private Servo TiltServoL;
  private Servo IntakeServo;
  // Slide motor limits

  @Override
  public void runOpMode() {
    // Initialize motors
    LFDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
    RFDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
    LRDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
    RRDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
    SlideMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "slide_motor");
    TiltServoR = hardwareMap.get(Servo.class, "right_tilt_servo");
    TiltServoL = hardwareMap.get(Servo.class, "left_tilt_servo");
    IntakeServo = hardwareMap.get(Servo.class, "intake_servo");
    // IntakeServo.setPosition(0);
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
    // IntakeServo.setDirection(Servo.Direction.REVERSE);

    // Slide Encoder
    // SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();

    if (opModeIsActive()) {

        TiltServoR.setPosition(0.0); // Neutral position
        TiltServoL.setPosition(0.025);
      while (opModeIsActive()) {
        // Drive control
        double y = gamepad1.left_stick_y;
        double x1 = gamepad1.left_stick_x;
        double x2 = gamepad1.right_stick_x;

        double LFPower = y - x2 - x1;
        double RFPower = y + x2 + x1;
        double LRPower = y - x2 + x1;
        double RRPower = y + x2 - x1;
        double SlidePower = gamepad1.left_trigger - gamepad1.right_trigger;

        LFDrive.setPower(LFPower);
        RFDrive.setPower(RFPower);
        LRDrive.setPower(LRPower);
        RRDrive.setPower(RRPower);

        if (gamepad1.x) {
          // Hover mode
          SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          SlideMotor.setTargetPosition(100); // Example target position
          ((DcMotorEx) SlideMotor).setVelocity(2100); // Use velocity for precise control

          if (!SlideMotor.isBusy()) {
            // Once the target is reached, switch back to manual control
            SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SlideMotor.setPower(0); // Ensure the motor stops before manual control
          }
        } else {
          // Manual control
          SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          SlideMotor.setPower(SlidePower);
        }

        // Tilt servo control
        if (gamepad1.dpad_up) {
          TiltServoR.setPosition(0.0); // Neutral position
          TiltServoL.setPosition(0.025);
        } 
        if (gamepad1.dpad_down) {
          TiltServoR.setPosition(0.35); // Tilted position
          TiltServoL.setPosition(0.375);
        }//end of if

        //Intake Servo Control
        
        if(gamepad1.a){
          TiltServoR.setPosition(-1);
          TiltServoL.setPosition(-1.025);
        }
        
        if(gamepad1.dpad_right){
          //Servo is Open
          IntakeServo.setPosition(0);
        }// end of if
        if(gamepad1.dpad_left){
          //Servo is closed
          IntakeServo.setPosition(0.15);
        }// end of if
      }
    }
  }
}
