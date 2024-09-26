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
    public static double[] pid1 = new double[]{0,0,0,0};
    public static double[] pid2 = new double[]{0,0,0,0};
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

        pid1 = new double[]{0.0001, 0, 0, 0};
        pid2 = new double[]{1, 1, 1, 1};

//        IntP.add(-55, pid1[0]);
//        IntP.add(365, pid1[0]);
//        IntI.add(-55,pid1[1]);
//        IntI.add(365, pid1[1]);
//        IntD.add(-55, pid1[2]);
//        IntD.add(365, pid1[2]);
//        IntF.add(-55,pid1[3]);
//        IntF.add(365, pid1[3]);

        IntP.add(-55, pid1[0]);
        IntP.add(365, pid2[0]);
        IntI.add(-55,pid1[1]);
        IntI.add(365, pid2[1]);
        IntD.add(-55, pid1[2]);
        IntD.add(365, pid2[2]);
        IntF.add(-55,pid1[3]);
        IntF.add(365, pid2[3]);

        IntP.createLUT(); IntI.createLUT(); IntD.createLUT(); IntF.createLUT();

        ff = 0; target = 0;

        Pid = new PIDFController(0,0,0,0);
    }

    @Override
    public void loop() {
        double armPos = armMotor.getCurrentPosition();
        double armDeg = ((armMotor.getCurrentPosition() * TICKS_PER_DEGREE) % 360);

//        Pid.setPIDF(
//                IntP.get(armDeg),
//                IntI.get(armDeg),
//                IntD.get(armDeg),
//                IntF.get(armDeg));

        Pid.setPIDF(pid1[0],pid1[1],pid1[2],pid1[3]);

        //TODO Add extension and get a extension PID - Kinda my whole idea behind using interpolation.

        if(gamepad2.a){ //Arm with Dashboard PID
            double output = Pid.calculate(armDeg, target); //Will be pid1 setpoint
            armMotor.setPower(output);
        }else if (gamepad2.y){
            double output = Pid.calculate(armPos,0); //pid[x] setpoint //TODO Use to get more pid value ranges
            armMotor.setPower(output);
        } else if (gamepad2.b){
            //armMotor.setPower(IntF.get(armMotor.getCurrentPosition()));
            armMotor.setPower(Math.cos(Math.toRadians(target/TICKS_PER_DEGREE)) * ff);
        } else if (gamepad2.x){
            IntP.createLUT(); IntI. createLUT(); IntD.createLUT(); IntF.createLUT();
        } else { //Not needed but to ensure that if there is no input the power is zero.
            armMotor.setPower(0);
        }

        FtcDashboard.getInstance().updateConfig();

        telemetry.addData("Encoder Pos:", armMotor.getCurrentPosition());
        telemetry.addData("Setpoint:", target);
        telemetry.addData("Deg", (armMotor.getCurrentPosition() * TICKS_PER_DEGREE) % 360);
        telemetry.addData("Output:", Pid.calculate(armPos,target));

        telemetry.addData("IntP:", IntP.get(armDeg));
        telemetry.addData("IntI:", IntI.get(armDeg));
        telemetry.addData("IntD:", IntD.get(armDeg));
        telemetry.addData("IntF:", IntF.get(armDeg));

        telemetry.addData("P:", Pid.getP());
        telemetry.addData("I:", Pid.getI());
        telemetry.addData("D:", Pid.getD());
        telemetry.addData("F:", Pid.getF());


        extender(0);
    }

    public void extender(double power){
        extenderLeft.setPower(power);
        extenderRight.setPower(power);
    }
}