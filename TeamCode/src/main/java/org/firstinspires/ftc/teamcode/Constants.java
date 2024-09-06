package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public final class Constants {
    PIDCoefficients drivePID = new PIDCoefficients();

    @Config
    public static class compRobot{
        public static HardwareMap hardwareMap;

        public static final DcMotor frontLeft = hardwareMap.dcMotor.get("FrontLeftDrive");
        public static final DcMotor backLeft = hardwareMap.dcMotor.get("BackLeftDrive");
        public static final DcMotor frontRight = hardwareMap.dcMotor.get("FrontRightDrive");
        public static final DcMotor backRight = hardwareMap.dcMotor.get("BackRightDrive");
    }
}

