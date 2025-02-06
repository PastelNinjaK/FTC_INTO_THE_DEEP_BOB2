package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TeleOp2425M3")
public class TeleOP2425M3 extends LinearOpMode {
  private DcMotor LFDrive;
  private DcMotor RFDrive;
  private DcMotor LRDrive;
  private DcMotor RRDrive;
  private DcMotor SlideMotorR;
  private DcMotor SlideMotorL;
  private Servo TiltServoR;
  private Servo TiltServoL;
  private CRServo IntakeServo;
  private double armDown = 0.0;
  private double armStraight = 0.2;
  private double armHover = 0.55;
  private double armFullTilt = 0.7;
  private double clawOn = 1.0;
  private double clawOff = -1.0;
  private boolean armIsDown = false;
  private boolean clawIsOn = false;
  // Slide motor limits

  @Override
  public void runOpMode() {
    // Initialize motors
    LFDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
    RFDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
    LRDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
    RRDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
    SlideMotorR = hardwareMap.get(DcMotor.class, "right_slide_motor");
    SlideMotorL = hardwareMap.get(DcMotor.class, "left_slide_motor");
    TiltServoR = hardwareMap.get(Servo.class, "right_tilt_servo");
    TiltServoL = hardwareMap.get(Servo.class, "left_tilt_servo");
    IntakeServo = hardwareMap.get(CRServo.class, "intake_servo");
    // Set motor brake behavior
    LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Reverse necessary motor directions
    SLideMotorL.setDirection(DcMotor.Direction.REVERSE);
    LRDrive.setDirection(DcMotor.Direction.REVERSE);
    RFDrive.setDirection(DcMotor.Direction.REVERSE);
    TiltServoR.setDirection(Servo.Direction.REVERSE);


    //Slide Encoder

    // SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    waitForStart();

    if (opModeIsActive()) {

        TiltServoR.setPosition(0.0); // Folded Position
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

        LFDrive.setPower(LFPower);
        RFDrive.setPower(RFPower);
        LRDrive.setPower(LRPower);
        RRDrive.setPower(RRPower);

        //Slide Control
        double SlidePower = gamepad1.left_trigger - gamepad1.right_trigger;

        SlideMotorR.setPower(SlidePower);
        SlideMotorL.setPower(SlidePower);



        // Tilt servo control & Hover Mode
        double intakePower = 0;
        double armPosition = 0;
        if (gamepad1.dpad_up) {
          armPosition = armStraight;
        }//end of if
        if (gamepad1.dpad_down) {
          armPosition = armHover;
        
        }//end of if
        if(gamepad1.left_bumper && !armIsDown){
          //arm goes down
          armPosition = armFullTilt;
          intakePower = clawOff;
          sleep(1200);
          
          //arm collects
          intakePower = clawOn;
          sleep(500);
          
          //arm goes back to hover mode
          armPosition = armHover;
        }

        TiltServoR.setPosition(armPosition);
        TiltServoL.setPosition(armPosition + 0.025);



        //Intake Servo Control
 
        if(gamepad1.dpad_right && !clawIsOn){
          //Servo is Open
          intakePower = clawOff;
          clawIsOn = true
        }// end of if
        if(gamepad1.dpad_left && clawIsOn){
          //Servo is closed
          intakePower = clawOn;
          clawIsOn = false
        }// end of if

        IntakeServo.setPower(intakePower);


        //Hover Mode
        

      }
    }
  }
}
