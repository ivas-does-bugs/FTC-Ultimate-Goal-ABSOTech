package org.firstinspires.ftc.teamcode;




        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Team 9960 Revision 161027.0
 * This program provides driver station control of the Team 9960 Mecanum Drive Prototype.
 *
 * This robot uses four VEX Mecanum wheels, each direct driven by Neverest 20 motors.
 * It is designed as a linear op mode, and uses RUN_WITH_ENCODER motor operation.
 *
 * The gamepad1 right joystick is used for translation movement, while the left joystick x-axis controls rotation.
 *
 */

@TeleOp(name="MecanumDriveEncoders", group="") // @Autonomous(...) is the other common choice
// @Disabled
public class TeleOP_Encoders extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorRightFront;
    private DcMotor motorLeftFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftBack;
    private DcMotorEx shooter;
    private DcMotor charger;
    private DcMotorEx pusher;
    private DcMotor brat;
    private Servo clesteDreapta;
    private Servo clesteStanga;

    double vertical;
    double rotate;
    double strafe;
    double pusherPower;
    double bratUP;
    double bratDOWN;
    double rotateSmalLeft;
    double rotateSmalRight;

    final double MAX_POS = 1.0;     // Maximum rotational position
    final double MIN_POS = 0.0;     // Minimum rotational position


    //variabile  encoder pusher
    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR = 1440;
    static final double COUNTS_PER_GRADE = COUNTS_PER_MOTOR / 360;
    static final double DRIVE_GEAR_REDUCTION = 2;        // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.93;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.5;
    static final double VITEZA_ARUNCARE = 930;
    // declare motor speed variables
    double RF; double LF; double RR; double LR;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;
    // operational constants
    double joyScale = 1.5;//0.5
    double motorMax = 1; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Motor initialization
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");   // motor stanga spate
        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");  // motor stanga fata
        motorRightFront = hardwareMap.dcMotor.get("RightFront");   // motor dreapta fata
        motorRightBack = hardwareMap.dcMotor.get("RightBack");   // motort drepata spate
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        //  pusher = hardwareMap.dcMotor.get("pusher");
        pusher = hardwareMap.get(DcMotorEx.class, "pusher");
        charger = hardwareMap.dcMotor.get("charger");
        brat = hardwareMap.dcMotor.get("brat");
        clesteDreapta = hardwareMap.get(Servo.class, "clesteDreapta");
        clesteStanga = hardwareMap.servo.get("clesteStanga");

        //Reverse Motors
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);

        //start the shooting thing
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        charger.setDirection(DcMotor.Direction.REVERSE);
        pusher.setDirection(DcMotorEx.Direction.REVERSE);

// set RunMode
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // Reset speed variables
            LF = 0; RF = 0; LR = 0; RR = 0;

            // Get joystick values
            Y1 = -gamepad1.right_stick_y * joyScale; // invert so up is positive
            X1 = gamepad1.right_stick_x * joyScale;
            Y2 = -gamepad1.left_stick_y * joyScale; // Y2 is not used at present
            X2 = gamepad1.left_stick_x * joyScale;

            // Forward/back movement
            LF += Y1; RF += Y1; LR += Y1; RR += Y1;

            // Side to side movement
            LF += X1; RF -= X1; LR -= X1; RR += X1;

            // Rotation movement
            LF += X2; RF -= X2; LR += X2; RR -= X2;

            // Clip motor power values to +-motorMax
            LF = Math.max(-motorMax, Math.min(LF, motorMax));
            RF = Math.max(-motorMax, Math.min(RF, motorMax));
            LR = Math.max(-motorMax, Math.min(LR, motorMax));
            RR = Math.max(-motorMax, Math.min(RR, motorMax));

            // Send values to the motors
            motorLeftFront.setPower(LF);
            motorRightFront.setPower(RF);
           motorLeftBack.setPower(LR);
            motorRightBack.setPower(RR);

            // Send some useful parameters to the driver station
            telemetry.addData("LF", "%.3f", LF);
            telemetry.addData("RF", "%.3f", RF);
            telemetry.addData("LR", "%.3f", LR);
            telemetry.addData("RR", "%.3f", RR);
        }
    }
}