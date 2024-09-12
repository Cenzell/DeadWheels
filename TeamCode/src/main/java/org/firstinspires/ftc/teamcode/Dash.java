package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.CameraStreamProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp(name="AS_DashTest")
public class Dash extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null; //Set a target

    public void init(){
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2); //TODO: Test for best value
        final CameraStreamProcessor cameraStream = new CameraStreamProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                //.addProcessor(aprilTag)
                .addProcessors(aprilTag, cameraStream)
                .build();

        //FtcDashboard.getInstance().startCameraStream(cameraStream, 30);
        dashboard.startCameraStream(cameraStream, 30);

        dashboard.updateConfig();
    }

    public static int dashInt = 0;

    public void loop(){

        telemetry.addData("DashInt", dashInt);
    }
}
