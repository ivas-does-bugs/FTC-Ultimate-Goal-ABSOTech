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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot {
    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveFront = null;
    public DcMotor brat = null;
    public DcMotor charger = null;
    public DcMotor pusher = null;

    public DcMotorEx shooter = null;


    public Servo clesteDreapta = null;
    public Servo clesteStanga = null;
    //public Servo servoDistance  = null;
    //public  ColorSensor sensorColor= null;
    //public DistanceSensor sensorRange= null;

    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    public static final double MAX_POS = 1.0;     // Maximum rotational position servo senzor de distanta
    public static final double MIN_POS = 0.0;     // Minimum rotational position


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "LeftBack");
        rightDrive = hwMap.get(DcMotor.class, "RightBack");
        rightDriveFront = hwMap.get(DcMotor.class, "RightFront");
        leftDriveFront = hwMap.get(DcMotor.class, "LeftFront");

        shooter = hwMap.get(DcMotorEx.class, "shooter");

        charger = hwMap.get(DcMotor.class, "charger");
        pusher = hwMap.get(DcMotor.class, "pusher");
        clesteDreapta = hwMap.get(Servo.class, "clesteDreapta");    //port 5
        clesteStanga = hwMap.get(Servo.class, "clesteStanga");      //port 4
        brat = hwMap.get(DcMotor.class, "brat");

        // Servo and Sensor intialization pentru senzor de distanta
        //sensorRange = hwMap.get(DistanceSensor.class, "sensorDistance");
        //servoDistance = hwMap.get(Servo.class, "servoDistance");


        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);

        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        /*
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        */
        //leftClaw.setDirection(Servo.Direction.REVERSE);


        brat.setDirection(DcMotorSimple.Direction.FORWARD);

        //Shooting and Charging and pushing
        //shooter.setDirection(DcMotor.Direction.REVERSE);
        charger.setDirection(DcMotor.Direction.FORWARD);
        pusher.setDirection(DcMotor.Direction.REVERSE);

        shooter.setDirection(DcMotorEx.Direction.REVERSE);


        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveFront.setPower(0);
        rightDriveFront.setPower(0);
        charger.setPower(0);
        shooter.setPower(0);
        pusher.setPower(0);
        brat.setPower(0);

        //leftArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        charger.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pusher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*Define and initialize ALL installed servos.
        leftClaw  = hwMap.get(Servo.class, "sst");
        rightClaw = hwMap.get(Servo.class, "sdr");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
        // initializare soenzor culoare
        sensorColor = hwMap.get(ColorSensor.class, "senzorCuloare");*/
        //servoDistance.setPosition(MIN_POS);


    }
}

