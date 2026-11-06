package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.GamepadHelper;

@Configurable
public class Intake {
    DcMotorEx intake; // first 3 rollers
    DcMotorEx transfer; // last roller

//    Servo nectarStopper;
//    Servo pollenStopper;

    OpMode op;
    GamepadHelper xbox;


    public static String intakeAction = "NULL";

    public Intake(OpMode opmode, GamepadHelper gamepad){
        op = opmode;
        xbox = gamepad;

        intake = op.hardwareMap.get(DcMotorEx.class, "intake");
        transfer = op.hardwareMap.get(DcMotorEx.class, "transfer");

//        nectarStopper = op.hardwareMap.get(Servo.class, "nectarStopper");
//        pollenStopper = op.hardwareMap.get(Servo.class, "pollenShooter");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void chooseAction(){
        // edge == 1 means the button was just pressed this loop
        // X = stop, A = intake, B = outtake everything, Y = shoot
        if(xbox.edge.get("x") == 1){
            intakeAction = "NULL";
        }
        if(xbox.edge.get("a") == 1){
            intakeAction = "INTAKE";
        }
        if(xbox.edge.get("b") == 1){
            intakeAction = "OUTAKE";
        }
        if(xbox.edge.get("y") == 1){
            intakeAction = "SHOOT";
        }
    }
    public void updateIntake(){
        chooseAction();
        if(intakeAction.equals("INTAKE")){
            intake();
        }
        else if(intakeAction.equals("NULL")){
            stop();
        }
        else if(intakeAction.equals("OUTAKE")){
            outake();
        }else if(intakeAction.equals("SHOOT")){
            shoot();
        }
    }


    public void intake(){
        intake.setPower(-1);
        transfer.setPower(0);

        // nectarStopper.setPosition(0);
        // pollenStopper.setPosition(0);
    }

    public void stop(){
        intake.setPower(0);
        transfer.setPower(0);
    }

    public void outake(){
        intake.setPower(1);
        transfer.setPower(-1);
    }

    public void shoot(){
        intake.setPower(-1);
        transfer.setPower(1);

//        nectarStopper.setPosition(1);
//        pollenStopper.setPosition(1);
    }

}
