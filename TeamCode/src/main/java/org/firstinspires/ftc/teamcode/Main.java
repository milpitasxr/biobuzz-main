package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class Main extends OpMode {
    Drivetrain dt;
    IMU imu;
    DcMotorEx intake, transfer;

    @Override
    public void init() {
        // --- IMU setup ---
        // Adjust logo/usb facing direction to match how your Control/Expansion Hub is actually mounted.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        // Create drivetrain instance
        dt = new Drivetrain(this, imu);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
    }

    @Override
    public void loop() {
        // Update drivetrain
        dt.update();

        // intake
        if(gamepad1.a){
            intake.setPower(-1);

        // stop everything
        } else if (gamepad1.b) {
            intake.setPower(0);
            transfer.setPower(0);

        // shoot
        } else if (gamepad1.y) {
            intake.setPower(-1);
            transfer.setPower(1);

        // outtake
        } else if (gamepad1.x){
            intake.setPower(1);
            transfer.setPower(-1);
        }


    }
}
