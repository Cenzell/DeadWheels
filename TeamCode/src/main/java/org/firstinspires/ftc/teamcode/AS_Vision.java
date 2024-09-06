package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.lib.CameraStreamProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AS_Vision extends OpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null; //Set a target

    @Override
    public void init() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2); //TODO: Test for best value
        final CameraStreamProcessor cameraStream = new CameraStreamProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                //.addProcessor(aprilTag)
                .addProcessors(aprilTag, cameraStream)
                .build();

        FtcDashboard.getInstance().startCameraStream(cameraStream, 30);
    }

    @Override
    public void loop() {
        telemetry.addData("April Tag Detect:", aprilTag.getDetections().toString());
    }
}
