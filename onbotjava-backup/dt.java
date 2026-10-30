package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class dt extends OpMode {
    DcMotorEx leftFront, rightFront, leftBack, rightBack;

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotorEx.class, "lF");
        rightFront = hardwareMap.get(DcMotorEx.class, "rF");
        leftBack   = hardwareMap.get(DcMotorEx.class, "lB");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rB");

        // Fix the two motors that were going backwards
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        // These two stay FORWARD
        // leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        // rightBack.setDirection(DcMotorEx.Direction.FORWARD);
    }

    @Override
    public void loop() {
        double x    =  gamepad1.left_stick_x;   // strafe
        double y    = -gamepad1.left_stick_y;   // forward (Y is inverted on gamepads)
        double turn =  gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

        double leftFrontPower  = (y + x + turn) / denominator;
        double rightFrontPower = (y - x - turn) / denominator;
        double leftBackPower   = (y - x + turn) / denominator;
        double rightBackPower  = (y + x - turn) / denominator;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(-rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(-rightBackPower);
    }
}