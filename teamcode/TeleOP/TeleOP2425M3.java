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
  private Servo RotateServo;
  private double armDown = 0.0;
  private double armStraight = 0.2;// This value needs to be modified
  private double armHover = 0.55;
  private double armFullTilt = 0.7;
  private double clawOpen = 0.1;
  private double clawClosed = 0;
  private double rotateCounterClockwise = -1.0;
  private double rotateClockwise = 1.0;
  private double rotateOff = 0.0;
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
    SlideMotorR = hardwareMap.get(DcMotor.class, "right_slide_motor");//This is the old slide motor
    SlideMotorL = hardwareMap.get(DcMotor.class, "left_slide_motor");//This is the new slide motor
    TiltServoR = hardwareMap.get(Servo.class, "right_tilt_servo");
    TiltServoL = hardwareMap.get(Servo.class, "left_tilt_servo");
    IntakeServo = hardwareMap.get(Servo.class, "intake_servo");//this is the claw servo
    RotateServo = hardWareMap.gey(CRServo.class, "rotate_servo");// this is the rotating servo
    // Set motor brake behavior
    LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    SlideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    SlideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    // Reverse necessary motor directions
    SLideMotorL.setDirection(DcMotor.Direction.REVERSE);
    LRDrive.setDirection(DcMotor.Direction.REVERSE);
    RFDrive.setDirection(DcMotor.Direction.REVERSE);
    TiltServoR.setDirection(Servo.Direction.REVERSE);
    //RotateServo.setDirection(CRServo.Direction.REVERSE);




    waitForStart();

    if (opModeIsActive()) {

      TiltServoR.setPosition(armDown); // Folded Position
      TiltServoL.setPosition(armDown +0.025);
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
          sleep(1200);// This needs to be adjusted

          //arm collects
          intakePower = clawOn;
          sleep(500);// this needs to be adjusted

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

        IntakeServo.setPosition(intakePower);


        // Rotating Servo
        double rotatePower = rotateOff;
        // if pressing x rotates the claw clockwise and pressing b rotates b counter clockwiss
        // go to line 62 and remove the // on that line.
        if(gamepad1.x){
          // claw rotates counter clockwise
          rotatePower = rotateLeft;
        }else if(gamepad1.b){
          // claw rotates clockwise
          rotatePower = rotateRight;
        }else{
          //claw stops rotating
          rotatePower = rotateOff;
        }//end of if

        RotateServo.setPower(rotatePower);
      }
    }
  }
}
