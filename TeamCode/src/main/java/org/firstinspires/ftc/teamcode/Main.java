package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.GamepadHelper;

@TeleOp
public class Main extends OpMode {
    Drivetrain dt;
    Intake intake;
    Shooter shooter;

    GamepadHelper xbox;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        xbox = new GamepadHelper(this);

        // Create drivetrain instance (field-centric, Pinpoint localizer)
        dt = new Drivetrain(this);
        intake = new Intake(this, xbox);
        shooter = new Shooter(this, xbox);


    }

    @Override
    public void loop() {
        // Read button edges once per loop, before any subsystem checks them
        xbox.update();

        dt.update();
        intake.updateIntake();
        shooter.updateShooter();

        // Send telemetry once per loop so subsystems don't overwrite each other's lines
        panelsTelemetry.update(telemetry);



    }
}
