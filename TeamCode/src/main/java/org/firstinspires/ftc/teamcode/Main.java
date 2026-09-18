package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

public class Main extends OpMode {
    Drivetrain dt;
    IMU imu;

    @Override
    public void init() {
        // --- IMU setup ---
        // Adjust logo/usb facing direction to match how your Control/Expansion Hub is actually mounted.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // Create drivetrain instance
        dt = new Drivetrain(this, imu);
    }

    @Override
    public void loop() {
        // Update drivetrain
        dt.update();
    }
}
