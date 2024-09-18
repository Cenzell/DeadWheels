package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="AS_FTCLibDrive")
public class AS_FTCLib_Odo extends OpMode {

    MotorEx frontLeft = new MotorEx(hardwareMap, "FrontLeftDrive");
    MotorEx frontRight = new MotorEx(hardwareMap, "FrontRightDrive");
    MotorEx backLeft = new MotorEx(hardwareMap, "BackLeftDrive");
    MotorEx backRight = new MotorEx(hardwareMap, "BackRightDrive");

    MotorEx
            encoderLeft = new MotorEx(hardwareMap, "RightEncoder"),
            encoderRight = new MotorEx(hardwareMap, "RightEncoder"),
            encoderPerp = new MotorEx(hardwareMap, "AuxEncoder");

    public static double TICKS_TO_INCHES = 8192 / ((35 / 25.4) * Math.PI);

    public static double TRACKWIDTH = 7.9;
    public static double CENTER_WHEEL_OFFSET = 10;

    // create the odometry object
    HolonomicOdometry holomonic = new HolonomicOdometry(
            encoderLeft::getDistance,
            encoderRight::getDistance,
            encoderPerp::getDistance,
            TRACKWIDTH, CENTER_WHEEL_OFFSET
    );

    // create the odometry subsystem
    OdometrySubsystem odometry = new OdometrySubsystem(holomonic);

    MecanumDrive mecanum = new MecanumDrive(frontLeft, frontRight,
            backLeft, backRight);

    @Override
    public void init() {

        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        holomonic.updatePose();
        mecanum.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, odometry.getPose().getHeading());
        if(gamepad1.a){
            holomonic.rotatePose(0);
        }
    }
}
