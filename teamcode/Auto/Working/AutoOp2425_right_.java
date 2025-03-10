package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name = "AutoOp2425_right_", group = "Robot")
public class AutoOp2425_right_ extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor armMotor;
    private CRServo wheelServo;
    private Servo wrist;

    // Constants for arm and wrist
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)
    double timePerTile = 0.180028069583241;
    double timePer90 = 0.3342359768;


    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        armMotor = hardwareMap.get(DcMotor.class, "left_arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        wheelServo = hardwareMap.get(CRServo.class, "intake");

        // Configure hardware
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(WRIST_FOLDED_IN);
        wheelServo.setPower(INTAKE_OFF);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()) {
            // Fill this up with the program
            program(1.0,1.0,timePerTile * 0.5,ARM_SCORE_SPECIMEN,WRIST_FOLDED_IN,INTAKE_OFF);//Robot turns on
            program(1.0,-1.0,timePer90,ARM_SCORE_SPECIMEN,WRIST_FOLDED_IN,INTAKE_OFF);//Robot rotates
            program(1.0,1.0,timePerTile * 1.5,ARM_SCORE_SPECIMEN,WRIST_FOLDED_IN,INTAKE_OFF);//Robot goes to high rung
            program(-1.0,1.0,timePer90,ARM_SCORE_SPECIMEN,WRIST_FOLDED_IN,INTAKE_OFF);
            program(0,0,timePerTile,ARM_SCORE_SPECIMEN,WRIST_FOLDED_IN,INTAKE_DEPOSIT);//Robot stops to score
            program(1.0,-1.0,timePer90,ARM_ATTACH_HANGING_HOOK,WRIST_FOLDED_OUT,INTAKE_OFF);//Robot rotates to go to low rung
            program(-1.0,-1.0,timePerTile * 2.5,ARM_ATTACH_HANGING_HOOK,WRIST_FOLDED_OUT,INTAKE_OFF);//Robot drives to low rung
            program(-1.0,1.0,timePer90,ARM_ATTACH_HANGING_HOOK,WRIST_FOLDED_OUT,INTAKE_OFF);//Robot rotates to go to low rung
            program(-1.0,-1.0,timePerTile * 0.5,ARM_ATTACH_HANGING_HOOK,WRIST_FOLDED_OUT,INTAKE_OFF);//Robot drives to low rung
            program(-1.0,1.0,timePer90,ARM_ATTACH_HANGING_HOOK,WRIST_FOLDED_OUT,INTAKE_OFF);//Robot rotates to go to low rung
            program(-1.0,-1.0,timePerTile * 0.5,ARM_ATTACH_HANGING_HOOK,WRIST_FOLDED_OUT,INTAKE_OFF);//Robot drives to low rung
            program(0,0,timePerTile,ARM_ATTACH_HANGING_HOOK,WRIST_FOLDED_OUT,INTAKE_OFF);//robot at low rung
            // Example telemetry feedback
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }
            telemetry.addData("Arm Target: ", armMotor.getTargetPosition());
            telemetry.addData("Arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    // Method to control all motors and servos
    public void program(double leftPower, double rightPower, double delay, double armPosition, double wristPosition, double intake) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        if (armMotor.getTargetPosition() != (int) armPosition) {
            armMotor.setTargetPosition((int) armPosition);
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        wrist.setPosition(wristPosition);
        wheelServo.setPower(intake);

        sleep((long) (delay * 1000));

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // Method to calculate time delay
    public double TimeDelay(double dist, double speed) {
        return dist / speed; // Distance in meters, speed in meters/second
    }
}
