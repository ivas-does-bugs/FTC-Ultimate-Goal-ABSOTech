package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auto Drive By Gyro", group = "")
//@Disabled
public class DriveByGyro extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU gyro = null;                    // Additional Gyro device
    Orientation lastAngles = new Orientation();
    double globalAngle;

    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.93;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double COUNTS_PER_MOTOR = 1440;
    static final double COUNTS_PER_GRADE = COUNTS_PER_MOTOR / 360;

    final double MAX_POS = 1.0;     // Maximum rotational position for Servo
    final double MIN_POS = 0.0;     // Minimum rotational position for Servo


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 1;     // 0.7Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.1;     // 0.15Larger is more responsive, but also less stable

    static final double V_ARUNCARE = 925;


    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);


        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //       robot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        WebCamera camera = new WebCamera();
        camera.init(hardwareMap, telemetry);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //********************** INITIALIZARE BRAT *************************


        cleste(MAX_POS);


//initializare IMU

        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        gyro.initialize(parameters);
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();


        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && !gyro.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Waiting for Start");
        telemetry.addData("IMU status", gyro.getCalibrationStatus().toString());
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading  %5.2f", lastAngles.firstAngle);
            telemetry.update();
        }

        //reset gyro


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn


        switch (camera.checkNumberOfRings()) {
            case 0:
                //****************************       TARGET A   *****************************
                //Pornim motorul de aruncare si mergem spre pozita de tragere
                aruncare(V_ARUNCARE);
                gyroDrive(DRIVE_SPEED, 53, 0.0);    // Drive FWD 55 inches
                gyroHold(TURN_SPEED, 0.0, 2);    // Hold 0 Deg heading for a 3 second

                //tragere
                impingere(4);
                aruncare(0);

                //se duce lasa wobble golul
                gyroDrive(DRIVE_SPEED, 5, 0.0);
                brat(-280);
                sleep(500);
                cleste(MIN_POS);
                sleep(500);
                brat(180);



                gyroTurn(TURN_SPEED, 157.0);         // Turn  CW  to  145 Degrees
                gyroHold(TURN_SPEED, 157.0, 1.5);         //  Hold 1450 Deg heading for a 1/2 second
                brat(-169);
                gyroDrive(0.7, 37, 157.0);  // Drive FWD 12 inches at 45 degrees
                gyroHold(TURN_SPEED, 157.0, 1.5);    // Hold 145 Deg heading for a 1second
                cleste(MAX_POS);
                sleep(500);
                brat(25);
                sleep(1000);

                gyroTurn(TURN_SPEED, -15.0);         // Turn  CW  to   0 Degrees
                gyroHold(TURN_SPEED, -15.0, 0.5);         //  Hold 20 Deg heading for a 1/2 second
                gyroDrive(DRIVE_SPEED, 30.0, -15.0);  // Drive FWD 39 inches at -30 degrees
                cleste(MIN_POS);
                sleep(500);
                brat(112);

                gyroTurn(TURN_SPEED, 45.0);         // Turn  CW  to  45 Degrees
                gyroHold(TURN_SPEED, 45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
                gyroDrive(DRIVE_SPEED, 20.0, 45.0);  // Drive FWD 12 inches at 45 degrees

//********************END TARGET A*************


                break;
            case 1:
                //****************************       TARGET A   *****************************
                //Pornim motorul de aruncare si mergem spre pozita de tragere
                aruncare(V_ARUNCARE);
                gyroDrive(DRIVE_SPEED, 53, 0.0);    // Drive FWD 55 inches
                gyroHold(TURN_SPEED, 0.0, 2);    // Hold 0 Deg heading for a 3 second

                //tragere
                impingere(4);
                aruncare(0);

                break;
            case 4:
                gyroDrive(DRIVE_SPEED, 80, 0);
                telemetry.addLine("Cazul 4C");
                telemetry.update();
                break;


        }

        sleep(2000);

/*
 //test gyro
        gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
        gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
        gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
        gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches
*/


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int newLeftTargetFront;
        int newRightTargetFront;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;
            newLeftTargetFront = robot.leftDriveFront.getCurrentPosition() + moveCounts;
            newRightTargetFront = robot.rightDriveFront.getCurrentPosition() + moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftDriveFront.setTargetPosition(newLeftTargetFront);
            robot.rightDriveFront.setTargetPosition(newRightTargetFront);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);
            robot.leftDriveFront.setPower(speed);
            robot.rightDriveFront.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.  && robot.leftDriveFront.isBusy() && robot.rightDriveFront.isBusy()
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);
                robot.leftDriveFront.setPower(leftSpeed);
                robot.rightDriveFront.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.leftDriveFront.setPower(0);
            robot.rightDriveFront.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveFront.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftDrive.setPower(leftSpeed);
        robot.rightDrive.setPower(rightSpeed);
        robot.leftDriveFront.setPower(leftSpeed);
        robot.rightDriveFront.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     * @paramtargetAngle Desired angle (relative to global reference established at last Gyro Reset).
     */

    private double getAngle() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getAngle();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    private double checkDirection() {
        double correction, angle, gain = .10;
        angle = getAngle();
        if (angle == 0)
            correction = 0;
        else
            correction = -angle;
        return correction;

    }

    public void driveOn(double power, double distanta, double timp) {
        double corectie;
        ElapsedTime holdTimp = new ElapsedTime();
        holdTimp.reset();
        while (opModeIsActive() && (holdTimp.time() < timp)) {
            corectie = checkDirection();
            robot.leftDrive.setPower(power - corectie);
            robot.rightDrive.setPower(power + corectie);
            robot.leftDriveFront.setPower(power - corectie);
            robot.rightDriveFront.setPower(power + corectie);


        }
        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveFront.setPower(0);
    }

    //****************************************** BRAT **********************************
    public void brat(int grade) {
        robot.brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.brat.setTargetPosition((int) (grade * COUNTS_PER_GRADE));
        robot.brat.setPower(1.0);
        robot.brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //**********************************************************
    //*****************************CLESTE***********************************************
    public void cleste(double position) {
        robot.clesteStanga.setPosition(position);
        robot.clesteDreapta.setPosition(position);
    }
    //**********************************************************

    //********************************* aruncare ******************
    public void aruncare(double v_aruncare) {
        robot.shooter.setVelocity(v_aruncare);
    }


    //************************************
    public void impingere(int rotatii) {
        robot.pusher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pusher.setPower(1.0);
        int tinta;
        tinta = (int) (rotatii * 360 * COUNTS_PER_GRADE);
        robot.pusher.setTargetPosition(tinta);
        robot.pusher.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        while (opModeIsActive() && robot.pusher.isBusy() && runtime.seconds() < 5) {
            telemetry.addData("Valoare : Encoder", "Stare  %7d: ", robot.pusher.getCurrentPosition());
            telemetry.update();
        }
        robot.pusher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooter.setVelocity(0);


    }

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


        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  sleep(250);   // optional pause after each move
    }

}
