package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name = "AutoOP2425AT", group = "Robot")
public class AutoOP2425AT extends LinearOpMode {
  private DcMotor LFDrive;
  private DcMotor RFDrive;
  private DcMotor LRDrive;
  private DcMotor RRDrive;
  private DcMotor SlideMotor;
  private Servo TiltServoR;
  private Servo TiltServoL;


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
           program();
    
      }
    }

    public void program(double LFPower, double LRPower,double RFPower, double RRPower, double delay, double armPosition, double wristPosition, double intake) {
      LFDrive.setPower(LFPower);
      RFDrive.setPower(RFPower);
      LRDrive.setPower(LRPower);
      RRDrive.setPower(RRPower);


      if (armMotor.getTargetPosition() != (int) armPosition) {
        armMotor.setTargetPosition((int) armPosition);
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      }

      TiltServoR.setPosition(wristPosition);
      TiltServoL.setPosition(wristPosition + 0.025);
      wheelServo.setPower(intake);

      sleep((long) (delay * 1000));

      LFDrive.setPower(0);
      RFDrive.setPower(0);
      LRDrive.setPower(0);
      RRDrive.setPower(0);
    }
  }

}
