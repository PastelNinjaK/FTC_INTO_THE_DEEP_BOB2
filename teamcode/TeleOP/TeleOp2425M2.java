
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;



@TeleOp(name = "TeleOp2425M2")
public class TeleOp2425M2 extends LinearOpMode {
  private DcMotor LFDrive;
  private DcMotor RFDrive;
  private DcMotor LRDrive;
  private DcMotor RRDrive;
  private DcMotor SlideMotor;
  private Servo TiltServoR;
  private Servo TiltServoL;
  private static final int SLIDE_MIN_POSITION = 0;      // Adjust based on hardware
  private static final int SLIDE_MAX_POSITION = 1000;  // Adjust based on hardware

  // private Servo IntakeServo;

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
    IntakeServo.setPosition(0);// to be adjusted(OPEN)
    //Brake mode
    LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Reverse necessary motor direction
    LRDrive.setDirection(DcMotor.Direction.REVERSE);
    RFDrive.setDirection(DcMotor.Direction.REVERSE);
    TiltServoR.setDirection(Servo.Direction.REVERSE);



    //Slide Encoder
    SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();

    if (opModeIsActive()) {
      TiltServoR.setPosition(0);
      TiltServoL.setPosition(0.025);

      while (opModeIsActive()) {
        // Drive control
        double y = gamepad1.left_stick_y;
        double x1 = gamepad1.left_stick_x;
        double x2 = gamepad1.right_stick_x;
        //option 1
        double LFPower = y - x2 - x1;
        double RFPower = y + x2 + x1;
        double LRPower = y - x2 + x1;
        double RRPower = y + x2 - x1;
        //option 2
        // double LFPower = y - x1 - x2;
        // double RFPower = y + x1 + x2;
        // double LRPower = y - x1 + x2;
        // double RRPower = y + x1 - x2;
        double slidePower = gamepad1.left_trigger - gamepad1.right_trigger;
        int tilt = 0;
        // int intake = 0;

        LFDrive.setPower(LFPower);
        RFDrive.setPower(RFPower);
        LRDrive.setPower(LRPower);
        RRDrive.setPower(RRPower);
                // Slide motor control with limits
        double slidePower = gamepad1.left_trigger - gamepad1.right_trigger;
        int currentSlidePosition = SlideMotor.getCurrentPosition();

        if (slidePower > 0 && currentSlidePosition >= SLIDE_MAX_POSITION) {
          slidePower = 0; // Stop moving up
        }
        
        SlideMotor.setPower(slidePower);
        
        if(gamepad1.dpad_up){
          //servo is straight
          TiltServoR.setPosition(0);
          TiltServoL.setPosition(0.025);

        }//end of if
        if(gamepad1.dpad_down){
          //Servo is tilted
          TiltServoR.setPosition(0.35);
          TiltServoL.setPosition(0.375);

        }//end of if
        
        //Intake Servo Control
        
        // if(gamepad1.dppad_right){
        //   //Servo is Open
        //   IntakeServo.setPosition(0);
        // }// end of if
        // if(gamepad1.dpad_left){
        //   //Servo is closed
        //   IntakeServo.setPosition(0.4);
        // }// end of if
        
        
        
        
        // if(gamepad1.x){
        //   TiltServoR.setPosition(0.5);
        //   TiltServoL.setPosition(0.5);
        //   IntakeServo.setPosition(0);//Open
        //   TimeUnit.SECONDS.sleep(1);
        //   IntakeServo.setPosition(0.45);//Close
        //   TimeUnit.SECONDS.sleep(1);
        //   TiltServo.setPosition(0.35);
        //   TiltServoL.setPosition(0.375);
        // }//end of if

      }
    }
  }

}

