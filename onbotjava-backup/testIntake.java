package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testIntake extends OpMode{
    DcMotorEx intake;

    @Override
    public void init(){
        intake = hardwareMap.get(DcMotorEx.class, "shot");
    }
    
    @Override
    public void loop(){
        if (gamepad1.a){
            intake.setPower(1);
        }
        if (gamepad1.b){
            intake.setPower(-1);
        }
        if (gamepad1.y){
            intake.setPower(0);
        }

        telemetry.addData("Velocity", intake.getVelocity());
        telemetry.addData("Current", intake.getCurrent(CurrentUnit.AMPS));
    }
}