
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.AutoOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;



@AutoOp(name = "Auto2425")
public class Auto2425 extends LinearOpMode {
  private DcMotor leftDrive;
  private DcMotor rightDrive;
  private DcMotor armMotor;
  private CRServo wheelServo; // Declare the servo
  private Servo wrist;

  //Java function for programing the motors
  public void program(double leftPower, double rightPower, double delay){
    leftDrive.setPower(leftPower);
    rightDrive.setPower(rightPower);
    sleep((long) (1000 * delay));
  }//end of function

  public void TimeDelay(double dist, double speed){
    return dist/speed;
    //DIST AND SPEED NEED TO BE IN METERS AND METERS PER SECOND RESPECTIVELY!
  }
  // TimeDelay will replace the thrid parameter for program
  // distance for the full arena is 12ft/365.76cm/ 3.657m (I hate the impreal system)
  // we need to calculate the max sped of our robot.
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

      //Parking to observation zone
      //sleep time need calibration
      //Assuming we place our robot at F2 (open link from next line) and our robot has a maximum speed of 0.5m/s.
      // Image link: https://firstroboticsbc.org/ftc/ftc-team-resources/intothedeep-autonomous-programs/
      // image name: Figure 9.4 TILE placement
      // 1 tile = 2ft / 0.6096 m
      // 0.5 is NOT the max speed of our robot, it's just me guessing.
      // once we actually get the top speed of our robot, we use that value as seconnd parameter of the TimeDelay function.
      double time = TimeDelay(0.6096,0.5);// we only need 1 because this is measuring how long it takes for the robot to travel 1 tile.

      program(-1.0, -1.0, time); // Backwards
      program(-1.0, 1.0, 1);  // Rotate counter-clockwise (can't change this rn cuz we need to find the time it take for the robot to turn 90 degrees).
      program(1.0, 1.0, time);   // Forward (Time is multiplied by 4 because the variable time is calculating how long it take for the robot to travel across 1 Tile).
      program(1.0, -1.0, 1);  // Rotate for parking(Clockwise)(Same for the first rotate)
      program(1.0, 1.0, time);   // Forward to observation zone

      // Stop motors
      leftDrive.setPower(0);
      rightDrive.setPower(0);




    }
  }
}


