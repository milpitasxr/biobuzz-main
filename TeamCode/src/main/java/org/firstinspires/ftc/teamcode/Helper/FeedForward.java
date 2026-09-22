package org.firstinspires.ftc.teamcode.Helper;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.List;

@TeleOp
@Configurable
public class FeedForward extends OpMode {

    DcMotorEx shooter;
    public static double volt = 0;

    public double batteryVoltage = Double.MAX_VALUE;
    List<VoltageSensor> voltageSensors;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        voltageSensors = hardwareMap.getAll(VoltageSensor.class);

    }

    @Override
    public void loop() {
        double total = 0;
        for (VoltageSensor sensor : voltageSensors) total += sensor.getVoltage();
        batteryVoltage = total / voltageSensors.size();

        shooter.setPower(volt / batteryVoltage);
        telemetry.addData("Voltage Applied", volt);
        telemetry.addData("Velocity", shooter.getVelocity());
    }
}
