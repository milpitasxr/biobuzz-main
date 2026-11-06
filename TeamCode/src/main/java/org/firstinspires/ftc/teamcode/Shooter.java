package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Helper.GamepadHelper;
import com.bylazar.telemetry.PanelsTelemetry;


@Configurable
public class Shooter {
    DcMotorEx pollenShoot;
    Servo pollenHood;
    OpMode op;

    // Hub velocity PIDF (hub units, live-tunable in Panels)
    // F = 32767 / max ticks/sec; a 6000 RPM goBILDA (28 CPR) maxes near 2800 ticks/sec
    public static double kP = 16;
    public static double kI = 1;
    public static double kD = 0;
    public static double kF = 15;

    // last coefficients sent to the hub, so we only resend when Panels changes them
    private double sentP = Double.NaN, sentI = Double.NaN, sentD = Double.NaN, sentF = Double.NaN;

    // anti-RPM-drop parameters
    public static double DROP_THRESHOLD = 50;   // ticks/sec drop that triggers boost
    public static double DROP_BOOST = 4;      // extra power added during a shot in volts
    public static double FARZONE = 0.5;
    public boolean runFlywheel = false;
    public double batteryVoltage = 0;

    // Screamage Shooter shi
    public static double tarVel = 1800; // ticks per sec

    public static double tarHood = 0.4;

    GamepadHelper xbox;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public Shooter(OpMode opmode, GamepadHelper gamepad){
        op = opmode;
        xbox = gamepad;

        pollenShoot = op.hardwareMap.get(DcMotorEx.class, "pollen");

        pollenHood = op.hardwareMap.get(Servo.class, "pollenHood");

        pollenShoot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pollenShoot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pollenShoot.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Push the Panels gains to the hub's velocity loop, only when they change
    private void applyPIDF(){
        if (kP != sentP || kI != sentI || kD != sentD || kF != sentF) {
            pollenShoot.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            sentP = kP; sentI = kI; sentD = kD; sentF = kF;
        }
    }

    public void updateFlywheel(){
        applyPIDF();
        pollenHood.setPosition(tarHood);

        if(runFlywheel){
            pollenShoot.setVelocity(tarVel);
        }else {
            // setPower(0) with FLOAT lets the flywheel coast down instead of braking
            pollenShoot.setPower(0);
        }
    }

    public void updateShooter(){
        if (xbox.edge.get("dpad_right") == 1){
            runFlywheel = !runFlywheel;
        }

        updateFlywheel();

        double current = pollenShoot.getVelocity();

        panelsTelemetry.addData("Flywheel on", runFlywheel);
        // If this number doesn't change when you edit tarVel in Panels, the edit isn't reaching the robot
        panelsTelemetry.addData("tarVel (robot)", tarVel);
        // Near +/-1 means the hub is maxed out and can't follow the target (wiring or gains)
        panelsTelemetry.addData("motor power", pollenShoot.getPower());
        panelsTelemetry.addData("current", current);
        panelsTelemetry.addData("error", tarVel - current);
        panelsTelemetry.update(op.telemetry);
    }
}
