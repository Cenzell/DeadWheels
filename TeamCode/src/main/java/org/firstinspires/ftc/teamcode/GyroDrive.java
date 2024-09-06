package org.firstinspires.ftc.teamcode;

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

@TeleOp(name="GyroDrive")
public class GyroDrive extends OpMode {

    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor leftEncoder, rightEncoder, auxEncoder;

    private Odometry Odometry;

    private IMU gyro = null;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null; //Set a target

    @Override
    public void init() {
        //Drive Motor Setup - Used the name from FeverDream
        frontLeft = hardwareMap.dcMotor.get("FrontLeftDrive");
        backLeft = hardwareMap.dcMotor.get("BackLeftDrive");
        frontRight = hardwareMap.dcMotor.get("FrontRightDrive");
        backRight = hardwareMap.dcMotor.get("BackRightDrive");

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        /*
        //Dead Wheel Setup
        leftEncoder = hardwareMap.get(DcMotor.class, "L");
        rightEncoder = hardwareMap.get(DcMotor.class, "R");
        auxEncoder = hardwareMap.get(DcMotor.class, "A");

        //Setup and start Odometry Thread
        Odometry = new Odometry(
                leftEncoder,
                rightEncoder,
                auxEncoder,
                0,
                0,
                0);
        Thread odometryThread = new Thread(Odometry);
        odometryThread.start(); */

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);

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
        //double robotHeading = Math.toRadians(Odometry.heading()); //Might need degrees???
        double robotHeading = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double h = gamepad2.right_stick_x;

        double finalX = x * Math.cos(robotHeading) - y * Math.sin(robotHeading);
        double finalY = x * Math.sin(robotHeading) + y * Math.cos(robotHeading);

        double denominator = Math.max(Math.abs(finalY) + Math.abs(finalX) + Math.abs(h), 1);
        double FLPower = (finalY + finalX + h) / denominator;
        double BLPower = (finalY - finalX + h) / denominator;
        double FRPower = (finalY - finalX - h) / denominator;
        double BRPower = (finalY + finalX - h) / denominator;

        frontLeft.setPower(FLPower);
        backLeft.setPower(BLPower);
        frontRight.setPower(FRPower);
        backRight.setPower(BRPower);

        telemetry.addData("April Tag Detect:", aprilTag.getDetections().toString());

        telemetry.addData("X Pos:", Odometry.xCoord());
        telemetry.addData("Y Pos", Odometry.yCoord());
        telemetry.addData("Heading:", robotHeading);
    }
}
