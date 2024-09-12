package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "AS_ColorTest")
public class ColorTest extends OpMode {

    RevColorSensorV3 colorSensor;

    TelemetryPacket packet = new TelemetryPacket();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init(){
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSense");
    }

    @Override
    public void loop(){
        //colorSensor = Constants.compRobot.colorSensor; //Might crash the Rev Controller :(

        float scale = 256; int min = 0, max = 255;

        double R = (colorSensor.alpha() * colorSensor.red() + (255 – colorSensor.red()) Bg_R) / 255

//        double alpha = Range.clip((int)(colorSensor.alpha() * scale), min, max);
//        double red = Range.clip((int)(colorSensor.red() * scale), min, max);
//        double green = Range.clip((int)(colorSensor.green() * scale), min, max);
//        double blue = Range.clip((int)(colorSensor.blue() * scale), min, max);

        colorSensor.enableLed(true);

        telemetry.addData("red:", colorSensor.getNormalizedColors().red);
        telemetry.addData("green:", colorSensor.getNormalizedColors().);
        telemetry.addData("blue:", colorSensor.getNormalizedColors().toColor());// I love carter tollman

        if(SampleDet(30,30,7,15,colorSensor)){
            telemetry.addData("Yellow", true);
        }
        if(SampleDet(4,10,10,10,colorSensor)){
            telemetry.addData("Blue", true);
        }
        if(SampleDet(20,8,1,5,colorSensor)){
            telemetry.addData("Red", true);
        }

        //packet.addLine("red: " + colorSensor.red() + " green: " + colorSensor.green() + " blue: " + colorSensor.blue());
        //dashboard.sendTelemetryPacket(packet); //Might error out unless ported into FTCDashboard project.
        //dashboard.updateConfig();
        //Ftc Dashboard might need a vpn to work - Kinda what fixed it on reveal day - Blame Securly
    }

    private boolean SampleDet(int r, int g, int b, double colorRange, ColorSensor colorSensor){
        if((colorSensor.red() <= r + colorRange && colorSensor.red() >= r - colorRange) || r == -1){
            //telemetry.addData("Red Range:", true);
            if((colorSensor.blue() <= b + colorRange && colorSensor.blue() >= b - colorRange) || b == -1){
                //telemetry.addData("Blue Range:", true);
                if((colorSensor.green() <= g + colorRange && colorSensor.green() >= g - colorRange) || g == -1){
                    //telemetry.addData("Green Range:", true);
                    //telemetry.addLine("Sample detected");
                    return true;//too eepy
                }
            }
        }
        return false;
    }
}
