package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorTest extends OpMode {

    ColorSensor colorSensor;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init(){
        colorSensor = Constants.compRobot.colorSensor; //Might crash the Rev Controller :(

        colorSensor.enableLed(true);
    }

    @Override
    public void loop(){

        //TODO input filtering - Avoid rev hub crash
        telemetry.addData("red:", colorSensor.red());
        telemetry.addData("green:", colorSensor.green());
        telemetry.addData("blue:", colorSensor.blue());

        SampleDet(100,50,100,50,colorSensor);

        packet.addLine("red: " + colorSensor.red() + " green: " + colorSensor.green() + " blue: " + colorSensor.blue());
        FtcDashboard.getInstance().sendTelemetryPacket(packet); //Might error out unless ported into FTCDashboard project.
        //Ftc Dashboard might need a vpn to work - Kinda what fixed it on reveal day - Blame Securely
    }

    private boolean SampleDet(int r, int g, int b, double colorRange, ColorSensor colorSensor){
        if((colorSensor.red() <= r + colorRange && colorSensor.red() >= r - colorRange) || r == -1){
            telemetry.addData("Red Range:", true);
            if((colorSensor.blue() <= b + colorRange && colorSensor.blue() >= b - colorRange) || b == -1){
                telemetry.addData("Blue Range:", true);
                if((colorSensor.green() <= g + colorRange && colorSensor.green() >= g - colorRange) || g == -1){
                    telemetry.addData("Green Range:", true);
                    telemetry.addLine("Sample detected");
                    return true;//too eepy
                }
            }
        }
        return false;
    }
}
