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
        double x1 = gamepad1.left_stick_x;
        double x2 = gamepad.right_stick_x;
        double LFPower;
        double RFPower;
        double LRPower;
        double RRPower;
        double slidePower = gamepad1.right_trigger - gamepad1.left_trigger;
        int wristServo = 0;
        if(gamepad1.a){
          wristServo++;
        }// end of if
        if ( x2 != 0) {
          // turn
          LFPower = y - x1;
          RFPower = y + x1;
          LRPower = y - x1;
          RRPower = y + x1;
        }//end of if
        if (x1 > 0){
          // move right
          LFPower = x1;
          RFPower = -x1;
          LRPower = -x1;
          RRPower = x1;
        }else if (x1 < 0){
          LFPower = -x1;
          RFPower = x1;
          LRPower = x1;
          RRPower = -x1;
        }



        LFDrive.setPower(LFPower);
        RFDrive.setPower(RFPower);
        LRDrive.setPower(LRPower);
        RRDrive.setPower(RRPower);
        SlideMotor.setPower(slidePower);
        //insert arm and servo code here



      }
    }
  }

}



