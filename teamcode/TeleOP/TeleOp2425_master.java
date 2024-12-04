package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name = "TeleOp2425")
public class TeleOp2425 extends LinearOpMode {
  private DcMotor leftDrive;
  private DcMotor rightDrive;
  private DcMotor armMotor;
  private CRServo wheelServo; // Declare the servo
  private Servo wrist;

  @Override
  public void runOpMode() {
    // Initialize motors
    leftDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
    rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
    armMotor = hardwareMap.get(DcMotor.class, "left_arm");
    wrist = hardwareMap.get(Servo.class, "wrist");

    // Initialize servo
    wheelServo = hardwareMap.get(CRServo.class, "intake");
    wrist.setPosition(0.45);

    // Reverse necessary motor directions
    rightDrive.setDirection(DcMotor.Direction.REVERSE);
    armMotor.setDirection(DcMotor.Direction.REVERSE);


;
    waitForStart();

    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // Drive control
        double z = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double leftPower = z - turn;
        double rightPower = z + turn;
        double armPower = gamepad1.right_trigger - gamepad1.left_trigger;

        if (armPower == 0) {
          if(gamepad1.dpad_up || gamepad1.dpad_right){
           armPower = -0.1;
          }else{
            armPower = 0.075;
          }
        }

        if (gamepad1.left_bumper) {
          wrist.setPosition(0.45);
        } else if (gamepad1.right_bumper) {
          wrist.setPosition(0.125);
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        armMotor.setPower(armPower);

        // Servo control
        double servoPosition = wheelServo.getPower();

        if (gamepad1.dpad_left) {
          servoPosition -= 100;
        } else if (gamepad1.dpad_right) {
          servoPosition += 100;
        } else if (gamepad1.dpad_up) {
          servoPosition += 50;
        } else if (gamepad1.dpad_down) {
          servoPosition -= 50;
        } else {
          servoPosition = 0;
        }


        wheelServo.setPower(servoPosition);
      }
    }
  }
}



