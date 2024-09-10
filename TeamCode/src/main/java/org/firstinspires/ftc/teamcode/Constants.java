package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@SuppressWarnings("unused")
@Config
public final class Constants{
    PIDCoefficients drivePID = new PIDCoefficients();

    public static HardwareMap hardwareMap;

    @Config
    public static class compRobot{
        //TODO: Get working/tested - Will help clean up code

        //Dashboard testing - Anything* with @Config should be a param on the dashboard
        public static int pleaseShowUpOnTheDashboard = 1;
        public static String pleaseShowUpOnTheDashboard2 = "2";

        public static final DcMotor frontLeft = hardwareMap.dcMotor.get("FrontLeftDrive");
        public static final DcMotor backLeft = hardwareMap.dcMotor.get("BackLeftDrive");
        public static final DcMotor frontRight = hardwareMap.dcMotor.get("FrontRightDrive");
        public static final DcMotor backRight = hardwareMap.dcMotor.get("BackRightDrive");

        public static final ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "ColorSense");
    }

    @Config
    public static class testBed{
        public static final DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        public static final DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        public static final DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        public static final DcMotor backRight = hardwareMap.dcMotor.get("backRight");
    }
}

