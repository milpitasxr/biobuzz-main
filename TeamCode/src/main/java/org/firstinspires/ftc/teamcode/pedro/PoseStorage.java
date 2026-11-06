package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

/**
 * Hands the robot's field pose from Auto to TeleOp.
 * Static fields survive between OpModes as long as the robot isn't restarted,
 * so TeleOp starts knowing exactly where Auto left the robot.
 */
public class PoseStorage {
    public static Pose currentPose = new Pose(0, 0, 0);
}
