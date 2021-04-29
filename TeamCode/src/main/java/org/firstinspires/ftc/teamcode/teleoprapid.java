package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "colectareTest", group = "")

public class teleoprapid extends LinearOpMode {


    private DcMotor motorRightFront;
    private DcMotor motorLeftFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftBack;
    private DcMotor colect;
    double power;
    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR = 1440;
    static final double COUNTS_PER_GRADE = COUNTS_PER_MOTOR / 360;
    static final double DRIVE_GEAR_REDUCTION = 2;        // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.93;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {

        //Motor initialization
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");   // motor stanga spate
        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");  // motor stanga fata
        motorRightFront = hardwareMap.dcMotor.get("RightFront");   // motor dreapta fata
        motorRightBack = hardwareMap.dcMotor.get("RightBack");   // motort dreapta spate

        colect = hardwareMap.dcMotor.get("charger");
        waitForStart();

        int leftInches = 1;
        int newLeftTarget = 0;

        while (opModeIsActive()) {
            newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           motorLeftFront.setPower(1);
           motorLeftFront.setTargetPosition(newLeftTarget);
            while (motorLeftFront.isBusy()) {
                motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Posioton ", motorLeftFront.getCurrentPosition());
                telemetry.update();
            }

        }


    }
}

