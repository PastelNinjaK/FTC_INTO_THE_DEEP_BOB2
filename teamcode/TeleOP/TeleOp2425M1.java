package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
  private Servo intake; // Declare the servo
  private Servo wrist;

  @Override
  public void runOpMode() {
    // Initialize motors
    LFDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
    RFDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
    LRDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
    RRDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
    SlideMotor = hardwareMap.get(DcMotor.class, "slide_motor");
    wrist = hardwareMap.get(Servo.class, "wrist_servo");
    intake = hardwareMap.get(Servo.class, "intake_servo");
    wrist.setPosition(0);// to be adjusted
    //Brake mode
    LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Reverse necessary motor direction


;
    waitForStart();

    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // Drive control
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double LFPower;
        double RFPower;
        double LRPower;
        double RRPower;
        double slidePower;
        int drive = 0;
        if(gamepad1.a){
          drive++;
        }// end of if
        if(drive % 2 == 0) {
          if ( x != 0 && y != 0) {
            // Rotate right
            LFPower = y - x;
            RFPower = y + x;
            LRPower = y - x;
            RRPower = y + x;
          }//end of if
        }else if(drive % 2 == 1){
          if( x < 0) {
            //move left
            LFPower = -x;
            RFPower = x;
            LRPower = x;
            RRPower = -x;
          } else if ( x > 0) {
            //move right
            LFPower = x;
            RFPower = -x;
            LRPower = -x;
            RRPower = x;
          } else if (y != 0){
            //forward
            LFPower = y;
            RFPower = y;
            LRPower = y;
            RRPower = y;
          }//end of if
        }// end of if


        LFDrive.setPower(LFPower);
        RFDrive.setPower(RFPower);
        LRDrive.setPower(LRPower);
        RRDrive.setPower(RRPower);
        //insert arm and servo code here



      }
    }
  }

}



