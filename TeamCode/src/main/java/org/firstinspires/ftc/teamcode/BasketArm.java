package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@TeleOp(name="AS_BasketTest")
public class BasketArm extends OpMode {
    @Override
    public void init() {
        DcMotor extender = hardwareMap.get(DcMotor.class, "extender");
        PIDCoefficients extPid = new PIDCoefficients(0,0,0);
        //PIDController extController =
    }

    @Override
    public void loop() {

    }
}
