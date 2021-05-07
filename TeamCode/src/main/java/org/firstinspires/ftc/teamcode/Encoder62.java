package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutonomDefect", group = "")
@Disabled
public class Encoder62 extends LinearOpMode {

    /* Declare OpMode members. */

    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR = 1440;
    static final double COUNTS_PER_GRADE = COUNTS_PER_MOTOR / 360;
    static final double DRIVE_GEAR_REDUCTION = 2;        // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.93;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.5;

    final double MAX_POS = 1.0;     // Maximum rotational position for Servo
    final double MIN_POS = 0.0;     // Minimum rotational position for Servo

    @Override
    public void runOpMode() {

        //Intializating the hardware
        robot.init(hardwareMap);

        //camera initalization
        WebCamera camera = new WebCamera();
        camera.init(hardwareMap, telemetry);

        //Signal that the initalization of engines have started
        telemetry.addData("Status", "THe robot is preparing");
        telemetry.update();

        //Reset of main engines
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Initializare", "Starting at %7d :%7d : %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.leftDriveFront.getCurrentPosition(),
                robot.rightDriveFront.getCurrentPosition());
        telemetry.update();

        //Robot has prepared the encoders
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        cleste(MAX_POS);
        sleep(1000);
        brat(15);

        telemetry.addData("Status", "The robot is getting ready");
        telemetry.update();
        aruncare(true);
        // Wait for the game to start (driver presses PLAY)
        // while (!opModeIsActive()) {
        //     telemetry.addLine("Number of rings: " + Integer.toString(camera.checkNumberOfRings()));
        //     telemetry.update();
        //  }

        waitForStart();
        /***********************************************************************
         * aici puneti instructiunile la autonom
         * - Ivas 12/03/2021
         ************************************************************************/

        //urgenta();

        switch (camera.checkNumberOfRings()) {
            case 0:
                camera = null;
                targetZoneA();
                break; //TODO CAZUL 0A
            case 1:
                camera = null;
                targetZoneB();
                break; //TODO CAZUL 1B
            case 4:
                camera = null;
                targetZoneC();
                break; //TODO CAZUL 4C
            default:
                camera = null;
                targetZoneA();
                break; //TODO CAZUL 0A
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();


    }
    //Semnal for the end of autnom
/*
    public void urgenta() {


        robot.leftDriveFront.setPower(1.0);
        robot.rightDriveFront.setPower(1.0);
        robot.leftDrive.setPower(1.0);
        robot.rightDrive.setPower(1.0);
        sleep(1000);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveFront.setPower(0);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        impingere();
        robot.leftDriveFront.setPower(1.0);
        robot.rightDriveFront.setPower(1.0);
        robot.leftDrive.setPower(1.0);
        robot.rightDrive.setPower(1.0);
        sleep(100);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveFront.setPower(0);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        brat(-90);
        cleste(MIN_POS);
        sleep(1500);
        brat(180);
        robot.rightDriveFront.setPower(1.0);
        robot.leftDrive.setPower(1.0);
        sleep(1000);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveFront.setPower(0);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

    }
*/

    public void encoderDrive(double speed, double leftInches, double rightInches, double leftFront, double rightFront) {

        int newLeftTarget = 0;
        int newRightTarget = 0;
        int newLeftFront = 0;
        int newRightFront = 0;

        // Determine new target position, and pass to motor controller
        newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = (int) (rightInches * COUNTS_PER_INCH);

        newLeftFront = (int) (leftFront * COUNTS_PER_INCH);
        newRightFront = (int) (rightFront * COUNTS_PER_INCH);

        //Semnal that path is in progress
        telemetry.addData("Path", "Running to %7d :%7d", newLeftTarget, newRightTarget);
        telemetry.update();

        // reset the timeout time and start motion.

        robot.leftDrive.setPower(Math.abs(speed));
        robot.leftDriveFront.setPower(Math.abs(speed));
        robot.rightDrive.setPower(Math.abs(speed));
        robot.rightDriveFront.setPower(Math.abs(speed));

        robot.leftDrive.setTargetPosition(newLeftTarget);
        robot.leftDriveFront.setTargetPosition(newLeftFront);
        robot.rightDrive.setTargetPosition(newRightTarget);
        robot.rightDriveFront.setTargetPosition(newRightFront);

        // Turn On RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy() || robot.leftDriveFront.isBusy() || robot.rightDriveFront.isBusy())) {

            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Spate", "Running to  %7d :%7d :",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition());

            telemetry.addData("Fata", "Running at %7d : %7d",
                    robot.leftDriveFront.getCurrentPosition(),
                    robot.rightDriveFront.getCurrentPosition());

            telemetry.addData("Target", robot.leftDriveFront.getTargetPosition());
            telemetry.update();
        }
        /* Stop all motion;
        //frana
        //robot.leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


       // robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //zero power
      //  robot.leftDriveFront.setPower(0);

      //  robot.rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // sleep(250);   // optional pause after each move
        telemetry.addData("Initializare", "Starting at %7d : :%7d",
                robot.leftDriveFront.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());

        telemetry.update();
*/
        //Turn of the power
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveFront.setPower(0);

        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  sleep(250);   // optional pause after each move
    }

    public void encoderDriveDiagonala(double speed, double leftInches, double rightFront) {

        int newLeftTarget = 0;

        int newRightFront = 0;

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active


        // Determine new target position, and pass to motor controller
        newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);


        newRightFront = (int) (rightFront * COUNTS_PER_INCH);


        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightFront);
        telemetry.update();

        // reset the timeout time and start motion.
        robot.leftDrive.setPower(Math.abs(speed));


        robot.rightDriveFront.setPower(Math.abs(speed));

        robot.leftDrive.setTargetPosition(newLeftTarget);


        robot.rightDriveFront.setTargetPosition(newRightFront);


        // Turn On RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() && (robot.leftDrive.isBusy() && robot.rightDriveFront.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightFront);
            telemetry.addData("Spate", "Running to  %7d  :", robot.leftDrive.getCurrentPosition());
            telemetry.addData("Fata", "Running at %7d ", robot.rightDriveFront.getCurrentPosition());

            telemetry.update();
        }

        // Stop all motion;
        //frana
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //zero power
        robot.leftDrive.setPower(0);
        robot.rightDriveFront.setPower(0);


        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // sleep(250);   // optional pause after each move
        telemetry.addData("Initializare", "Starting at %7d : :%7d",
                robot.leftDrive.getCurrentPosition(),


                robot.rightDriveFront.getCurrentPosition());


        telemetry.update();
    }

    public void encoderDriveDiagonalaDreapta(double speed, double leftInches, double rightFront) {

        int newLeftTarget = 0;
        int newRightFront = 0;

        robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active


        // Determine new target position, and pass to motor controller
        newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
        newRightFront = (int) (rightFront * COUNTS_PER_INCH);


        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightFront);
        telemetry.update();

        // start motion
        robot.leftDriveFront.setPower(Math.abs(speed));
        robot.rightDrive.setPower(Math.abs(speed));

        robot.leftDriveFront.setTargetPosition(newLeftTarget);
        robot.rightDrive.setTargetPosition(newRightFront);


        // Turn On RUN_TO_POSITION
        robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() && (robot.leftDriveFront.isBusy() && robot.rightDrive.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightFront);
            telemetry.addData("Spate", "Running to  %7d  :", robot.leftDriveFront.getCurrentPosition());
            telemetry.addData("Fata", "Running at %7d ", robot.rightDrive.getCurrentPosition());

            telemetry.update();
        }

        // Stop all motion;
        //frana
        robot.leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //zero power
        robot.leftDriveFront.setPower(0);
        robot.rightDrive.setPower(0);


        // Turn off RUN_TO_POSITION
        robot.leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // sleep(250);   // optional pause after each move
        telemetry.addData("Initializare", "Starting at %7d : :%7d",
                robot.leftDriveFront.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());

        telemetry.update();
    }

    public void colectare(boolean isON) {
        if (isON) {
            robot.charger.setPower(1);
        } else {
            robot.charger.setPower(0);
        }
    }

    public void aruncare(boolean isON) {
        double viteza_aruncare = 850;
        if (isON) {
            robot.shooter.setVelocity(viteza_aruncare);
        } else {
            robot.shooter.setVelocity(0);
        }
    }

    public void cleste(double position) {
        robot.clesteStanga.setPosition(position);
        robot.clesteDreapta.setPosition(position);
    }

    public void impingere() {
        robot.pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pusher.setPower(1.0);
        int tinta;
        tinta = (int) (90 * COUNTS_PER_GRADE);
        tinta = -tinta;
        for (int k = 1; k <= 8; k++) {
            if (k % 2 != 0) tinta = -1 * tinta;
            else tinta = -1 * tinta;

            robot.pusher.setTargetPosition(tinta);
            robot.pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // run until the end of the match (driver presses STOP)
            runtime.reset();

            while (opModeIsActive() && robot.pusher.isBusy() && runtime.seconds() < 5) {
                telemetry.addData("Valoare K: Encoder", "Stare %1d %7d: ", k, robot.pusher.getCurrentPosition());
                telemetry.update();
            }
            robot.pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }

    public void impingereRapida() {
        robot.pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pusher.setPower(1.0);
        int tinta;
        tinta = (int) (90 * COUNTS_PER_GRADE);
        tinta = -tinta;
        for (int k = 1; k <= 4; k++) {
            if (k % 2 != 0) tinta = -1 * tinta;
            else tinta = -1 * tinta;

            robot.pusher.setTargetPosition(tinta);
            robot.pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // run until the end of the match (driver presses STOP)
            runtime.reset();

            while (opModeIsActive() && robot.pusher.isBusy() && runtime.seconds() < 5) {
                telemetry.addData("Valoare K: Encoder", "Stare %1d %7d: ", k, robot.pusher.getCurrentPosition());
                telemetry.update();
            }
            robot.pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }

    public void brat(int grade) {
        robot.brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.brat.setTargetPosition((int) (grade * COUNTS_PER_GRADE));
        robot.brat.setPower(1.0);
        robot.brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //pentru strafe stanga encoderDrive(+, -, -, +)

    public void targetZoneA() {

        //porneste spre pozitia de tragere

        encoderDrive(DRIVE_SPEED, 53, 53, 53, 53);
        impingere();
        aruncare(false);
        //lasa wobble golul jos
        cleste(MIN_POS);
        sleep(350);
        //ridica bratul ca sa nu loveasca wobble golul
        brat(60);

        //robotul se intoarce si se intreabta spre al 2 lea wobble gol
        encoderDrive(TURN_SPEED, -38, 38, -38, 38);//rotire
        brat(-85);
        encoderDrive(0.5, 34, 34, 34, 36);//FATA

        //robtul prinde al doilea wobble gol
        cleste(MAX_POS);
        sleep(600);
        brat(100);


        //robotul se intoarce sa duca al doilea robot
        encoderDrive(TURN_SPEED, 43, -43, 43, -43);//rotire
        encoderDrive(0.5, 36, 36, 36, 36);//FATA

        brat(-75);
        sleep(500);
        cleste(MIN_POS);
        brat(100);
        sleep(500);

        //parcare
        encoderDrive(0.8, 20, -20, -20, 20);//STANGA
        encoderDrive(0.8, 5, 5, 5, 5);
        brat(-70);

    }

    public void targetZoneB() {

        //porneste spre pozitia de tragere

        encoderDrive(0.8, 53, 53, 53, 53);//FATA
        impingere();

        aruncare(false);
        //duce si lasa wobble golul
        encoderDrive(0.5, 17, -17, -17, 17);//STANGA
        encoderDrive(0.6, 27, 27, 27, 27);//FATA

        brat(-15);
        cleste(MIN_POS);
        sleep(350);

        //colecteza inel
        colectare(true);
        encoderDrive(0.8, -48, -48, -48, -48);//SPATE


        //se roteste ia wobble goal
        encoderDrive(0.7, -40, 40, -40, 40);//rotire
        colectare(false);
        encoderDrive(0.5, 6, 6, 6, 6);//FATA

        cleste(MAX_POS);
        sleep(350);
        brat(15);

        //se roteste duce wobble gol
        encoderDrive(0.6, 40, -40, 40, -40);//rotire
        encoderDrive(0.7, 50, 50, 50, 50);//FATAd
        aruncare(true);
        //se pregates se retarge
        cleste(MIN_POS);
        sleep(350);
        brat(130);

        //se duce in poztioa de tragere 2
        encoderDrive(0.7, -17, -17, -17, -17);//SPATE
        impingereRapida();

        //parcheaza
        brat(-50);
        cleste(MIN_POS);

        encoderDrive(1.0, 12, 12, 12, 12);//FATA


    }


    public void targetZoneC() {


        encoderDrive(0.8, 53, 53, 53, 53);
        impingere();
        aruncare(false);
        //livreaza wobble goal
        encoderDrive(1, 50, 50, 50, 50);

        cleste(MIN_POS);
        sleep(400);
        brat(100);

        //se retrage spre inele
        encoderDrive(0.5, 16, -16, -16, 16);

        //daramai nelel
        encoderDrive(1, -64, -64, -64, -64);

        colectare(true);
        encoderDrive(0.5, -14, -14, -14, -14);

        //se duce sa trage
        encoderDrive(DRIVE_SPEED, 16, 16, 16, 16);
        aruncare(true);
        encoderDrive(TURN_SPEED, -22, 22, 22, -22);

        impingere();
        colectare(false);
        //parchekaza
        encoderDrive(DRIVE_SPEED, 18, 18, 18, 18);
    }
}


