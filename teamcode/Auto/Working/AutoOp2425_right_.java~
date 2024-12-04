package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;
    final double INTAKE_OFF = 0.0;

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

        waitForStart();

        if (opModeIsActive()) {
            // Fill this up with the program
            program(1.0,1.0,1,ARM_COLLAPSED_INTO_ROBOT,WRIST_FOLDED_IN,INTAKE_OFF);
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
