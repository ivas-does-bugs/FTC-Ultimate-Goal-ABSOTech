package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TeleOp_Principal", group = "")

public class ModulTeleOP extends OpMode {

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

    private ElapsedTime runtime = new ElapsedTime();
    //variabile  encoder pusher
    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR = 1440;
    static final double COUNTS_PER_GRADE = COUNTS_PER_MOTOR / 360;
    static final double DRIVE_GEAR_REDUCTION = 2;        // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.93;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.5;
    static final double VITEZA_ARUNCARE = 990;


    @Override
    public void init() {

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


        // brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
    }


    @Override
    public void loop() {


        //Get input

        vertical = -gamepad1.right_stick_y;
        strafe = gamepad1.left_stick_x;

        rotateSmalLeft = (double) gamepad1.left_trigger;
        rotateSmalRight = (double) gamepad1.right_trigger;

        if (rotateSmalLeft > 0)
            rotate = rotateSmalLeft * -0.3;
        else if (rotateSmalRight > 0)
            rotate = rotateSmalRight * 0.3;
        else
            rotate = gamepad1.right_stick_x;// pe joystickul din dreapta x-ul e inversat 4 no rreal reason at all
        //
        //

        bratUP = (double) (gamepad2.left_trigger);
        bratDOWN = (double) (gamepad2.right_trigger);

        brat.setPower((bratUP - bratDOWN)*0.5);

        //  pusher.setPower(pusherPower);

        telemetry.addLine(Double.toString(gamepad1.right_stick_x));
        telemetry.update();

        //Set basic Motor to input
        motorRightFront.setPower(vertical - rotate - strafe);
        motorRightBack.setPower(vertical - rotate + strafe);
        motorLeftFront.setPower(vertical + rotate + strafe);
        motorLeftBack.setPower(vertical + rotate - strafe);


        //Turn on/off the charging mechanism
        if (gamepad2.a) {
            charger.setPower(1);
        }
        if (gamepad2.b) {
            charger.setPower(0.0);
        }

        if(gamepad2.dpad_down) {
            charger.setPower(-1);
        }

        //Turn on/off the shooting mechanism
        if (gamepad2.x) {
            shooter.setVelocity(VITEZA_ARUNCARE);

        }
        if (gamepad2.y) {
            shooter.setVelocity(0);
        }

        if (gamepad2.left_bumper) {
            double VA = 890;
            shooter.setVelocity(VA);
        }

        if (gamepad2.dpad_right) {
            clesteDreapta.setPosition(MAX_POS);
            clesteStanga.setPosition(MAX_POS);
        }
        if (gamepad2.dpad_left) {
            clesteDreapta.setPosition(MIN_POS);
            clesteStanga.setPosition(MIN_POS);
        }

        // setare pusher cu encoder
        if (gamepad2.right_bumper) {
            int tinta;

           pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            tinta = (int) (4 * 360 * COUNTS_PER_GRADE);


            pusher.setTargetPosition(tinta);
            pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pusher.setPower(1);

            runtime.reset();
        }
    }
        /*
                while (opModeIsActive() && pusher.isBusy()) {
                    telemetry.addData("Valoare Encoder", "Stare  %7d: ", pusher.getCurrentPosition());
                    telemetry.update();
                }
          */
    //pusher.setPower(0);


    //Update console

                /*
                if (gamepad2.dpad_up) {
                    pusher.setPower(1);
                }

                if (gamepad2.dpad_down) {
                    pusher.setPower(-1);
                }
                pusher.setPower(0); */
/*
                int tinta;
                if(gamepad2.left_bumper) {
                    pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    pusher.setPower(1.0);

                    tinta = (int)(70 * COUNTS_PER_GRADE);
                    pusher.setTargetPosition(tinta);
                    pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if(pusher.getCurrentPosition() >= ( (int)  70 * COUNTS_PER_GRADE -10) ){
                    tinta = (int)(70 * COUNTS_PER_GRADE);
                    pusher.setTargetPosition(-tinta);
                    pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }


 */
/*

                if(gamepad2.dpad_down) {
                    pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    pusher.setPower(1.0);
                    int tinta;
                    tinta = (int) (70 * COUNTS_PER_GRADE);
                    tinta = -tinta;
                    for (int k = 1; k <= 8; k++) {
                        if (k % 2 != 0) tinta = -1 * tinta;
                        else tinta = -1 * tinta;

                        pusher.setTargetPosition(tinta);
                        pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        // run until the end of the match (driver presses STOP)
                        runtime.reset();

                        while (opModeIsActive() && pusher.isBusy() && runtime.seconds() < 5) {
                            telemetry.addData("Valoare K: Encoder", "Stare %1d %7d: ", k, pusher.getCurrentPosition());
                            telemetry.update();
                        }
                        pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        sleep(250);

                    }
                }
*/


                /*if (gamepad1.x){
                    shooter.setPower(0.7);
                    sleep(4000);
                    pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    int tinta;
                    tinta =(int)(360 * COUNTS_PER_GRADE);
                    encoderDrive(DRIVE_SPEED, 58, 58, 58,58, 15);
                    encoderDrive(DRIVE_SPEED, -30, 30, 30,-30, 10);
                    pusher.setTargetPosition(tinta);
                    pusher.setPower(1);
                    pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(1000);
                    pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sleep(1000);
                    encoderDrive(DRIVE_SPEED, -8,8,8,-8,10);
                    pusher.setTargetPosition(tinta);
                    pusher.setPower(1);
                    pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(1000);
                    pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sleep(1000);
                    encoderDrive(DRIVE_SPEED, -8,8,8,-8,10);
                    pusher.setTargetPosition(tinta);
                    pusher.setPower(1);
                    pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(1000);
                    pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }*/
}
