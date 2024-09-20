package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="AS_Arm")
public class AS_Arm extends OpMode {

    InterpLUT IntP = new InterpLUT(), IntI = new InterpLUT(), IntD = new InterpLUT(), IntF = new InterpLUT();

    DcMotorEx armMotor;
    DcMotorEx extenderLeft, extenderRight; //TODO Check 560 ticks per rev
    //TODO Limit C=42/cos(x)


    PIDFController Pid;

    //Set in dashboard
    public static double[] pid1 = new double[]{1,1,1,1};
    public static double ff;
    public static double target;

    double TICKS_PER_DEGREE = 360.0/8192.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "BigArm");

        extenderLeft = hardwareMap.get(DcMotorEx.class, "ExtenderLeft");
        extenderRight = hardwareMap.get(DcMotorEx.class, "ExtenderRight");

        //TODO - Not sure which one needs to be reversed.
        //extenderLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //extenderRight.setDirection(DcMotorSimple.Direction.REVERSE);

        pid1 = new double[]{0.1, 0.1, 0.1, 0.1};

        IntP.add(pid1[0], -1);
        IntP.add(pid1[0], 1000);
        IntI.add(pid1[1],-1);
        IntI.add(pid1[1], 1000);
        IntD.add(pid1[2], -1);
        IntD.add(pid1[2],1000);
        IntF.add(pid1[3],-1);
        IntF.add(pid1[3], 1000);

        IntP.createLUT(); IntI.createLUT(); IntD.createLUT(); IntF.createLUT();

        ff = 0; target = 0;

        Pid = new PIDFController(0,0,0,0);
    }

    @Override
    public void loop() {
        double armPos = armMotor.getCurrentPosition();

        Pid.setPIDF(0.1,0.1,0.1,0.1);
                /*IntP.get(armPos),
                IntI.get(armPos),
                IntD.get(armPos),
                IntF.get(armPos))*/;

        //TODO Add extension and get a extension PID - Kinda my whole idea behind using interpolation.

        if(gamepad1.a){ //Arm with Dashboard PID
        double output = Pid.calculate(armPos,target); //Will be pid1 setpoint
            armMotor.setPower(output);
        }else if (gamepad1.y){
            double output = Pid.calculate(armPos,0); //pid[x] setpoint //TODO Use to get more pid value ranges
            armMotor.setPower(output);
        } else if (gamepad1.b){
            //armMotor.setPower(IntF.get(armMotor.getCurrentPosition()));
            armMotor.setPower(Math.cos(Math.toRadians(target/TICKS_PER_DEGREE)) * ff);
        } else { //Not needed but to ensure that if there is no input the power is zero.
            armMotor.setPower(0);
        }

        telemetry.addData("Encoder Pos:", armMotor.getCurrentPosition());
        telemetry.addData("Setpoint:", target);
        telemetry.addData("Deg", (armMotor.getCurrentPosition() * TICKS_PER_DEGREE) % 360);

        /*telemetry.addData("IntP:", IntP.get(armPos));
        telemetry.addData("IntI:", IntI.get(armPos));
        telemetry.addData("IntD:", IntD.get(armPos));
        telemetry.addData("IntF:", IntF.get(armPos));*/

        extender(1);
    }

    public void extender(double power){
        extenderLeft.setPower(power);
        extenderRight.setPower(power);
    }
}
