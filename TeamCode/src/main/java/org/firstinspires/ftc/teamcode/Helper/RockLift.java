package org.firstinspires.ftc.teamcode.Helper;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp(name = "Rock Lift", group = "TeleOp")
public class RockLift extends LinearOpMode {

    // ---- Tunable in Panels (live) ----
    public static double MAX_UP_POWER = 0.8;    // power at full stick up
    public static double MAX_DOWN_POWER = 0.5;  // power at full stick down
    public static double DEADZONE = 0.05;       // ignore stick drift
    public static boolean FINE_CONTROL = true;  // squares the stick input for gentler control near center

    // ---- Tunable in Panels (applied on next INIT) ----
    // Flip if a winch spools the wrong way
    public static boolean REVERSE_1 = false;
    public static boolean REVERSE_2 = false;
    public static boolean REVERSE_3 = false;
    public static boolean REVERSE_4 = false;

    private final String[] NAMES = {"winch1", "winch2", "winch3", "winch4"};
    private final DcMotorEx[] winches = new DcMotorEx[4];

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        boolean[] reverse = {REVERSE_1, REVERSE_2, REVERSE_3, REVERSE_4};

        for (int i = 0; i < 4; i++) {
            winches[i] = hardwareMap.get(DcMotorEx.class, NAMES[i]);
            winches[i].setDirection(reverse[i]
                    ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
            winches[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // hold rocks in place when stick is released
            winches[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        panels.addData("Status", "Ready. Press START.");
        panels.update(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            // Stick up reads negative, so flip it: up = positive = raise
            double input = -gamepad1.left_stick_y;
            if (Math.abs(input) < DEADZONE) input = 0;

            if (FINE_CONTROL) input = input * Math.abs(input); // keeps sign, softens small movements

            double power = input > 0 ? input * MAX_UP_POWER : input * MAX_DOWN_POWER;

            for (DcMotorEx winch : winches) {
                winch.setPower(power);
            }

            panels.addData("Stick", input);
            panels.addData("Power", power);
            panels.update(telemetry);
        }
    }
}