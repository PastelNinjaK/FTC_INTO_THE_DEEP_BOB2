Conversation opened. 1 unread message.

Skip to content
Using Calgary Catholic School District Mail with screen readers
1 of 255
ALL CODE
Inbox

Aleksandr Gruzdev <aleksandrg1@learn.cssd.ab.ca>
4:18 PM (0 minutes ago)
to me

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Autonomous.drive.advanced.PoseStorage;

@TeleOp(name = "TeleOp2425")
public class TeleOp2425 extends LinearOpMode {

    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private IMU imu;
    private DcMotor sliderMotor;

    private Servo basketServo;

    private Servo clawArmServo;
    private Servo clawPivot;
    private CRServo clawServo;


    private double speedCoefficient = 0.75;
    private double turningCoefficient = 0.5;
    private double sliderNetPower;

    private boolean clawHovering = false;

    private double rightFrontPower;
    private double rightRearPower;
    private double leftFrontPower;
    private double leftRearPower;
    private SampleMecanumDrive drive;

    private boolean isForward = false;

    private double slideTargetCM = 105;
    private double slideHangCM = 39;
    private boolean slideHang = false;
    private boolean slideUp = false;
    final double TICKS_PER_REVOLUTION = 751.8;
    final double CIRCUMFERENCE = 24.4;



    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize motors

        rightRearMotor = hardwareMap.get(DcMotor.class, "backRight"); // right rear port 0
        leftRearMotor = hardwareMap.get(DcMotor.class, "backLeft"); // left rear port 1
        rightFrontMotor = hardwareMap.get(DcMotor.class, "frontRight"); // right front port 2
        leftFrontMotor = hardwareMap.get(DcMotor.class, "frontLeft"); // left front port 3
        sliderMotor = hardwareMap.get(DcMotor.class, "sliderMotor"); // slider port 0 on expansion hub 

        basketServo = hardwareMap.get(Servo.class, "basketServo");

        clawArmServo = hardwareMap.get(Servo.class,"clawArmServo");
        clawPivot = hardwareMap.get(Servo.class,"clawPivot");
        clawServo = hardwareMap.get(CRServo.class, "clawServo");


        //Set direction:
        //If these don't result in moving forward, switch to the other side of motors
        //TODO: UNCOMMENT IF USING MECANUM DRIVE
//        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //for field centric
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        if(opModeIsActive()){

            //for basket reset
            basketServo.setPosition(0);

            //for intake reset
            clawArmServo.setPosition(0.8);
            clawPivot.setPosition(0.35);

            //for claw reset
            clawServo.setPower(-0.1);
            sleep(1200);
            clawServo.setPower(0);


            while(opModeIsActive()){


                if(!isForward){
                    orienting(gamepad1);
                    sliderManual(gamepad2);
                    telemetry.update();
                    if(gamepad1.y){
                        drive = new SampleMecanumDrive(hardwareMap);
                        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        isForward = true;
                    }
                }

                else{
                    fieldCentricDrive(gamepad1);
                    moveSliderToPosition(gamepad2);
                    intakeFunction(gamepad1);
                    basketFunction(gamepad2);

                    //sliderManual(gamepad1);
                    //testingServos(gamepad1);
                    telemetry.update();
                }


            }
        }
    }

    public void intakeFunction(Gamepad gamepad) {
        telemetry.addData("claw servo power: ", clawServo.getPower());
        telemetry.addData("claw arm servo: ", clawArmServo.getPosition());
        telemetry.addData("claw pivot servo: ", clawPivot.getPosition());

//        //close claw
//        if (gamepad.a) {
//            clawServo.setPower(0.1);
//            sleep(1200);
//            clawServo.setPower(0);
//        }
//
//        //open claw
//        else if (gamepad.b) {
//            clawServo.setPower(-0.1);
//            sleep(1200);
//            clawServo.setPower(0);
//        }


//       claw arm down
        if(gamepad.right_bumper && !clawHovering){
//            clawArmServo.setDirection(Servo.Direction.REVERSE);
            clawArmServo.setPosition(0.16);
            clawPivot.setPosition(0);
            clawHovering = true;
            }

        //pick up sample and claw arm up
        else if (gamepad.left_bumper && clawHovering) {
            //arm down and pick up
            clawArmServo.setPosition(0);
            clawServo.setPower(0.1);
            sleep(1200);

            //arm up
            clawArmServo.setPosition(1);
            clawPivot.setPosition(0.35);
            sleep(1400);

            //dropping sample into bucket
            clawServo.setPower(-0.25);
            sleep(700);
            clawServo.setPower(0);

            //clear arm of slide
            clawArmServo.setPosition(0.8);

            clawHovering = false;
        }

        //undo hover
//        else if(gamepad.right_bumper && clawHovering){
//            clawArmServo.setDirection(Servo.Direction.REVERSE);
//            clawArmServo.setPosition(1);
//            clawPivot.setPosition(0);
//            clawServo.setPower(-0.25);
//            sleep(700);
//            clawServo.setPower(0);
//
//            clawHovering = false;
//        }
    }

    public void basketFunction(Gamepad gamepad){
        telemetry.addData("Basket Servo: ", basketServo.getPosition());

        if(gamepad.b && slideUp){
            basketServo.setPosition(0.2);
            sleep(600);
            basketServo.setPosition(0);
        }


    }


    public void fieldCentricDrive(Gamepad gamepad){

        //speed boost when holding left stick button
        if(gamepad.left_stick_button || gamepad.right_stick_button){
            speedCoefficient = 0.75;
            turningCoefficient = 0.75;
        }
        else{
            speedCoefficient = 0.2;
            turningCoefficient = 0.175;
        }

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad.left_stick_y * speedCoefficient,
                -gamepad.left_stick_x * speedCoefficient
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad.right_stick_x * turningCoefficient
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();
    }

    public void mecanumTest(Gamepad gamepad){
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details

        waitForStart();



            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad.left_stick_y,
                            -gamepad.left_stick_x,
                            -gamepad.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
    }

    public void orienting(Gamepad gamepad){
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y * 0.5,
                        -gamepad.left_stick_x * 0.5,
                        -gamepad.right_stick_x * 0.4
                )
        );
        drive.update();
    }

    //written as a test for kevlar tension
    public void sliderManual(Gamepad gamepad){
        int position = sliderMotor.getCurrentPosition();
        double slideCentimeters = CIRCUMFERENCE * (position/TICKS_PER_REVOLUTION);
        telemetry.addData("Encoder ticks: ", position);
        telemetry.addData("Slide cm: ", slideCentimeters);

        sliderNetPower = (gamepad.right_trigger * 0.5)-(gamepad.left_trigger * 0.5);

        sliderMotor.setPower(sliderNetPower);

    }

    public void moveSliderToPosition(Gamepad gamepad){

        int position = sliderMotor.getCurrentPosition();
        double slideCentimeters = CIRCUMFERENCE * (position/TICKS_PER_REVOLUTION);
        int slideTargetCM_toTicks = (int)((slideTargetCM / CIRCUMFERENCE) * TICKS_PER_REVOLUTION);
        int slideHangCM_toTicks = (int)((slideHangCM / CIRCUMFERENCE) * TICKS_PER_REVOLUTION);
        telemetry.addData("Encoder ticks: ", position);
        telemetry.addData("Slide cm: ", slideCentimeters);
        telemetry.addData("slideUp:", slideUp);
        telemetry.addData("slide power: ", sliderMotor.getPower());
        telemetry.addData("slide target: ", slideTargetCM_toTicks);

        //drop off sample
        if(gamepad.dpad_up){
            slideUp = true;
            slideHang = false;
        }

        //both off
        else if(gamepad.dpad_down){
            slideUp = false;
            slideHang = false;
        }

        //hang
        else if(gamepad.x){
            slideUp = false;
            slideHang = true;
        }



        if(slideUp){
            sliderMotor.setTargetPosition(slideTargetCM_toTicks);
        }

        else if (slideHang){
            sliderMotor.setTargetPosition(slideHangCM_toTicks);
        }

        else{
            sliderMotor.setTargetPosition(0);
        }
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setPower(0.75);

    }


    public void testingServos(Gamepad gamepad){
        telemetry.addData("servo 0", clawArmServo.getPosition());

        if (gamepad.dpad_left){
            clawArmServo.setPosition(0);
        }

        else if(gamepad.dpad_right){
            clawArmServo.setPosition(1);
        }
    }


}
