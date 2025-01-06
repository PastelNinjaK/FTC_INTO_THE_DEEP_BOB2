package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;



@TeleOp(name = "TeleOp2425M")
public class TeleOp2425M extends LinearOpMode {
  private DcMotor LFDrive;
  private DcMotor RFDrive;
  private DcMotor LRDrive;
  private DcMotor RRDrive;
  private DcMotor SlideMotor;
  private CRServo intake; // Declare the servo
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

    // Reverse necessary motor directions
    RFDrive.setDirection(DcMotor.Direction.REVERSE);
    RRDrive.setDirection(DcMotor.Direction.REVERSE);
    slideMotor.setDirection(DcMotor.Direction.REVERSE);


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
        double diagonal = phythagoras(x,y);
        int drive = 0;


// Movements
        if(gamepad1.a){
          drive++;
        }
        if(y != 0 && x == 0){
          // Forward and backward
          LFPower = y;
          RFPower = y;
          LRPower = y;
          RRPower = y;
        }else if(y > 0 && x > 0 && drive == 1){
          // Quadrant 1 movement
          LFPower = diagonal;
          RFPower = 0;
          LRPower = 0;
          RRPower = diagonal;
        }else if(y > 0 && x < 0 && drive == 1){
          // Quadrant 2 movement
          LFPower = 0;
          RFPower = diagonal;
          LRPower = diagonal;
          RRPower = 0;
        }else if(y < 0 && x < 0 && drive == 1){
          // Quadrant 3 movement
          LFPower = 0;
          RFPower = diagonal * -1;
          LRPower = diagonal * -1;
          RRPower = 0;
        }else if(y < 0 && x > 0 && drive == 1){
          // Quadrant 4 movement
          LFPower = diagonal * -1;
          RFPower = 0;
          LRPower = 0;
          RRPower = diagonal * -1;
        }else if(drive == 2){
          // Rotate left
          LFPower = x * -1;
          RFPower = x;
          LRPower = x * -1;
          RRPower = x;
        }else if(drive == 2){
          // Rotate right
          LFPower = x;
          RFPower = x * -1;
          LRPower = x;
          RRPower = x * -1;
        }// end of if

        LFDrive.setPower(LFPower);
        RFDrive.setPower(RFPower);
        LRDrive.setPower(LRPower);
        RRDrive.setPower(RRPower);
// Servo control

//        double servoPosition = wheelServo.getPower();
//
//        if (gamepad1.dpad_left) {
//          servoPosition -= 100;
//        } else if (gamepad1.dpad_right) {
//          servoPosition += 100;
//        } else if (gamepad1.dpad_up) {
//          servoPosition += 50;
//        } else if (gamepad1.dpad_down) {
//          servoPosition -= 50;
//        } else {
//          servoPosition = 0;
//        }
//
//
//        wheelServo.setPower(servoPosition);
      }
    }
  }

  public double squared(double base){
    return base*base;
  }
  public double squareroot(double num){
    return num/num;
  }
  public double phythagoras(double a,double b){
    return squareroot(squared(a)+squared(b));
  }
}



