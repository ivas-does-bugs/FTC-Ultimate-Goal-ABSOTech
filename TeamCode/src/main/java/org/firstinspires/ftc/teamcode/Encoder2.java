/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Pushbot: Auto Drive By Encoder dreapta", group = "Pushbot")

public class Encoder2 extends LinearOpMode {


    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2;        // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.93;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.5;

    //final double MAX_POS = 1.0;     // Maximum rotational position for distanceServo
    //final double MIN_POS = 0.0;     // Minimum rotational position for distanceServo


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Initializare", "Starting at %7d :%7d : %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.leftDriveFront.getCurrentPosition(),
                robot.rightDriveFront.getCurrentPosition());
        telemetry.update();


        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        /*encoderDrive(0.5,  -16,  -16,-16,-16, 5.0);
        *encoderDrive(DRIVE_SPEED,  11,  -11,-11,11, 5.0);  // lateral
        *encoderDrive(DRIVE_SPEED,  -15.5,  -15.5,-15.5,-15.5, 5.0);  //fata
        *robot.leftClaw.setPosition(1.0);            // prinde tava
        *robot.rightClaw.setPosition(0.0);
        *sleep(1000);     // pause for servos to move
        *encoderDrive(DRIVE_SPEED,  20,  20,20,20, 5.0);
        *encoderDrive(TURN_SPEED,   35, -35,35,-35, 9.0);  //roteste spre perete
        *encoderDrive(TURN_SPEED,   -15, -15,-15,-15, 9.0);// impinge
        *robot.leftClaw.setPosition(0.0);            // lasa tava
        *robot.rightClaw.setPosition(1.0);
        *sleep(1000);
        *
        * //encoderDrive(DRIVE_SPEED,-11,11,11,-11,5.0);//PARCARE POD
        * //encoderDrive(DRIVE_SPEED, 40,40,40,40,9.0);//PARCARE POD
        *
        *encoderDrive(TURN_SPEED, 15,-15,-15,15,5.0);//PARCARE LATERAL
        *encoderDrive(DRIVE_SPEED, 44,44,44,44,9.0);//PARCARE LATERAL
       
        encoderDrive(DRIVE_SPEED,  35,35,35,35, 5.0);//FATA
        encoderDrive(TURN_SPEED,  35,  -35,-35,35, 5.0);  // lateral stanga
       
       
       // encoderDrive(DRIVE_SPEED,  -15,  -15,-15,-15, 5.0);  //fata
       /* sleep(300);
        robot.leftClaw.setPosition(1.0); // prinde tava
        robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move
        encoderDrive(TURN_SPEED,   10, -10,10,-10, 9.0);  //roteste spre perete
        encoderDrive(TURN_SPEED,   15, 15,15,15, 9.0);
        encoderDrive(TURN_SPEED,   28, -28,28,-28, 9.0);
        encoderDrive(TURN_SPEED,   -20, -20,-20,-20, 9.0);// impinge
        robot.leftClaw.setPosition(0.0);            // lasa tava
        robot.rightClaw.setPosition(1.0);
        sleep(1000);
        encoderDrive(DRIVE_SPEED,7,-7,-7,7,5.0);//PARCARE POD
        encoderDrive(DRIVE_SPEED, 41,41,41,41,9.0);//PARCARE POD
        //encoderDrive(TURN_SPEED, 15,-15,-15,15,5.0);//PARCARE LATERAL
        //encoderDrive(DRIVE_SPEED, 44,44,44,44,9.0);//PARCARE LATERAL
        
          // S3: Reverse 24 Inches with 4 Sec timeout
//robot.leftDrive.getMotorType().getMaxRPM()

        encoderDrive(DRIVE_SPEED, 59, 59, 59, 59, 5.0);//FATA
        encoderDrive(TURN_SPEED, 14, -14, -14, 14, 5.0);//STANGA

        telemetry.addData("Path", "Complete");
        telemetry.update();


        /*
         *  Method to perfmorm a relative move, based on encoder counts.
         *  Encoders are not reset as the move is based on the current position.
         *  Move will stop if any of three conditions occur:
         *  1) Move gets to the desired position
         *  2) Move runs out of time
         *  3) Driver stops the opmode running.
         */
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double leftFront, double rightFront, double timeoutS) {

        int newLeftTarget = 0;
        int newRightTarget = 0;
        int newLeftFront = 0;
        int newRightFront = 0;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = (int) (rightInches * COUNTS_PER_INCH);

        newLeftFront = (int) (leftFront * COUNTS_PER_INCH);
        newRightFront = (int) (rightFront * COUNTS_PER_INCH);


        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
        telemetry.update();

        // reset the timeout time and start motion.
        runtime.reset();
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
        while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.leftDriveFront.isBusy() && robot.rightDrive.isBusy())) {

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

        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftDriveFront.setPower(0);
        robot.rightDriveFront.setPower(0);

            /* break
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
             robot.leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            */

        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  sleep(250);   // optional pause after each move

    }
}
    /*
   public int checkNumberOfRings(){
               
    
       Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)robot.sensorRange;
       int nr=0;
       double nr_min, nr_max;
       
     // while (opModeIsActive()){

 if(robot.sensorRange.getDistance(DistanceUnit.INCH) > 15) {
                 telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));
                telemetry.addData(">>", "No ring detected, procedding to zone A");
                nr=0;;
            } else {
                telemetry.addData(">>", "Ring detected, testing for more rings");
                nr_min=robot.sensorRange.getDistance(DistanceUnit.INCH);
               
                robot.servoDistance.setPosition(MAX_POS);
                sleep(600);
                 nr_max=robot.sensorRange.getDistance(DistanceUnit.INCH);
                 
                if(nr_max <=15) {
                     telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));
                    telemetry.addData(">>", "4 rings detected, proceeding to zone C");
                   nr=4;
                    
                } else {
                    telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));
                    telemetry.addData(">>", "Only 1 ring detected, proceeding to zone B");
                    nr= 1;
                }
            }
            
   
      telemetry.update();
       //sleep(4200);
      return nr;
     // }//while 
    
   }
   
  /* 
   public void recharge(boolean isOn){
    if(isOn){
      robot.charger.setPower(1);
   }else{
      robot.charger.setPower(0);
   }
   }
   public void shoot(boolean isOn){
    if(isOn){
      robot.shooter.setPower(0.56);
   }else{
      robot.shooter.setPower(0);
   
   }
}
*/


    
