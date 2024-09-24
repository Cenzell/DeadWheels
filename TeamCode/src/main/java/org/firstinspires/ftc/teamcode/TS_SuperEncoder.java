package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SuperDeadwheel")
public class TS_SuperEncoder extends OpMode {

    DcMotor encoder;

    @Override
    public void init() {
        encoder = hardwareMap.get(DcMotor.class, "climber");
    }

    @Override
    public void loop() {
        telemetry.addData("Encoder:", encoder.getCurrentPosition());
    }
}
