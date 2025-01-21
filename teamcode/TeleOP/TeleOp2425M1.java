package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;



@TeleOp(name = "TeleOp2425M1")
public class TeleOp2425M1 extends LinearOpMode {
  private DcMotor LFDrive;
  private DcMotor RFDrive;
  private DcMotor LRDrive;
  private DcMotor RRDrive;
  private DcMotor SlideMotor;
  private Servo TiltServoR;
  private Servo TiltServoL;

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
    // IntakeServo = hardwareMap.get(Servo.class, "intake_servo");
    // IntakeServo.setPosition(0);// to be adjusted(OPEN)
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



    ;
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
        SlideMotor.setPower(slidePower);
        // //insert arm and servo code here

        // if(gamepad1.a){
        //   tilt++;
        // }// end of if
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
        // if(gamepad1.x){
        //   TiltServo.setPosition(0.6);
        //   IntakeServo.setPosition(0);//Open
        //   TimeUnit.SECONDS.sleep(1);
        //   IntakeServo.setPosition(0.45);//Close
        //   TimeUnit.SECONDS.sleep(1);
        //   TiltServo.setPosition(0.45);
        // }//end of if

      }
    }
  }

}


