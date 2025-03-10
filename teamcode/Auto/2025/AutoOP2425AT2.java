package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Autonomous(name = "AutoOP2425AT2", group = "Robot")
public class AutoOP2425AT2 extends LinearOpMode {
  private DcMotor LFDrive;
  private DcMotor RFDrive;
  private DcMotor LRDrive;
  private DcMotor RRDrive;
  private DcMotor SlideMotor;
  private Servo TiltServoR;
  private Servo TiltServoL;
  private Servo IntakeServo;
  //final double low_rung = 578.7116112967108;
  //final double high_rung = 1070.7473672576434;
  //final double low_basket = 762.9220420998186;
  //final double high_basket = 1296.1627628456572;
  // final double low_chamber = 362.9915015404398;
  final double high_chamber = 2600;
  final double slide_off = 10;
  double Straight = 0;
  double Tilt = 0.35;
  double On = 0;
  double Off = 0.15;



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
    // IntakeServo.setPosition(0);// to be adjusted(OPEN)
    //Brake mode
    LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Reverse necessary motor direction
    
    LFDrive.setDirection(DcMotor.Direction.REVERSE);
    RRDrive.setDirection(DcMotor.Direction.REVERSE);
    TiltServoR.setDirection(Servo.Direction.REVERSE);
    
    SlideMotor.setDirection(DcMotor.Direction.REVERSE);
    SlideMotor.setTargetPosition(10);
    SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ;
    waitForStart();

    if (opModeIsActive()) {
      program(0,0,0,0,1,high_chamber,Straight,On);
      program(1,1,1,1,1,high_chamber,Straight,On);
      program(0,0,0,0,1,high_chamber,Straight,Off);
      program(-1,-1,-1,-1,1,slide_off,Straight,Off);
      program(-1,1,1,-1,1,slide_off,Straight,Off);
      
      

    }
  }

  public void program(double LFPower, double LRPower,double RFPower, double RRPower, double delay, double slidePosition, double wristPosition, double intake) {
    LFDrive.setPower(LFPower*0.5);
    RFDrive.setPower(RFPower*0.5);
    LRDrive.setPower(LRPower*0.5);
    RRDrive.setPower(RRPower*0.5);;



    SlideMotor.setTargetPosition((int) slidePosition);
    ((DcMotorEx) SlideMotor).setVelocity(9000);


    TiltServoR.setPosition(wristPosition);
    TiltServoL.setPosition(wristPosition+0.0195);
    IntakeServo.setPosition(intake);

    sleep((long) (delay * 1000));

    LFDrive.setPower(0);
    RFDrive.setPower(0);
    LRDrive.setPower(0);
    RRDrive.setPower(0);
  }
}
