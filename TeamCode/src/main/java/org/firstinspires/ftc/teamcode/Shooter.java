package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.PIDController;

public class Shooter {
    DcMotorEx pollen;
    DcMotorEx nectar;
    Servo pollenHood;
    Servo nectarHood;
    OpMode op;

    // PID constants get stored per-controller, not static
    public static double Kp = 0.08;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double maxI = Double.MAX_VALUE;
    PIDController pid = new PIDController(Kp, Ki, Kd, maxI);

    // anti-RPM-drop parameters
    public static double DROP_THRESHOLD = 50;   // ticks/sec drop that triggers boost
    public static double DROP_BOOST = 4;      // extra power added during a shot in volts
    public static double FARZONE = 0.5;
    public boolean runFlywheel = false;
    public double batteryVoltage = 0;


    public Shooter(OpMode opmode){
        op = opmode;
        pollen = op.hardwareMap.get(DcMotorEx.class, "pollen");
        nectar = op.hardwareMap.get(DcMotorEx.class, "nectar");

        pollenHood = op.hardwareMap.get(Servo.class, "pollenHood");
        nectarHood = op.hardwareMap.get(Servo.class, "nectarHood");

        pollen.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        nectar.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);


    }
}
