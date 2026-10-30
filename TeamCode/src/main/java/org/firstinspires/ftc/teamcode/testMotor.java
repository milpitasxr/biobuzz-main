package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Configurable
public class testMotor extends OpMode{
    DcMotorEx intake;

    public static double power = 1;

    @Override
    public void init(){
        intake = hardwareMap.get(DcMotorEx.class, "shot");
    }

    @Override
    public void loop(){
        if (gamepad1.a){
            intake.setPower(power);

        }
        if (gamepad1.b){
            intake.setPower(-power);
        }
        if (gamepad1.y){
            intake.setPower(0);
        }

        telemetry.addData("Voltage", intake.getPower());
        telemetry.addData("Velocity", intake.getVelocity());
        telemetry.addData("Current", intake.getCurrent(CurrentUnit.AMPS));
    }
}