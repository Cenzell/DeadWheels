package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorTest extends LinearOpMode {

    ColorSensor colorSensor;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode(){
        colorSensor = Constants.compRobot.colorSensor; //Might crash the Rev Controller :(

        colorSensor.enableLed(true);

        telemetry.addData("red:", colorSensor.red());
        telemetry.addData("green:", colorSensor.green());
        telemetry.addData("blue:", colorSensor.blue());

        packet.addLine("red: " + colorSensor.red() + " green: " + colorSensor.green() + " blue: " + colorSensor.blue());
        FtcDashboard.getInstance().sendTelemetryPacket(packet); //Might error out unless ported into FTCDashboard project.
        //Ftc Dashboard might need a vpn to work - Kinda what fixed it on reveal day - Blame Securly
    }
}
