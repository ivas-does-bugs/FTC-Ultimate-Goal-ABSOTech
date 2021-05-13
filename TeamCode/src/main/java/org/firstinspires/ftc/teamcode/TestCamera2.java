package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "CameraTest")
public class TestCamera2 extends LinearOpMode {

    @Override
    public void runOpMode() {

        WebCamera  camera = new WebCamera();
        camera.init(hardwareMap, telemetry);

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addLine(Integer.toString(camera.checkNumberOfRings()));
            telemetry.update();
        }

    }

}
