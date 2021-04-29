package org.firstinspires.ftc.teamcode;

import android.app.UiAutomation;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.logging.StreamHandler;

public class WebCamera {

    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;

    final int FOUR_RING_THRESHOLD = 140;
    final int ONE_RING_THRESHOLD = 130;

    //local op members
    HardwareMap hwMap = null;
    Telemetry telemetry = null;
    private ElapsedTime period = new ElapsedTime();


    public void init(HardwareMap ahwMap, Telemetry atelemetry) {
        //reference to hardwareMap
        hwMap = ahwMap;
        telemetry = atelemetry;

        pipeline = new SkystoneDeterminationPipeline();
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Camera Intializated");
        telemetry.update();
    }

    public int checkNumberOfRings() {
        while (pipeline.getAnalysis() == 0)
            ;
        //telemetry.addLine(Integer.toString(pipeline.getAnalysis()));
        if(pipeline.getAnalysis() < ONE_RING_THRESHOLD) { return 0;}
        else if(pipeline.getAnalysis() < FOUR_RING_THRESHOLD) {return 1;}
        else if(pipeline.getAnalysis() >= FOUR_RING_THRESHOLD) {return 4;}

        return 0;
    }

}

