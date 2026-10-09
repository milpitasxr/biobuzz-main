package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.IOException;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Example: call finder.getPathToBall() whenever you want a path to the closest ball. */
@TeleOp(name = "Ball Distance Example", group = "Vision")
public class BallDistanceExample extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    BallFinder finder;
    try {
      finder = new BallFinder(hardwareMap, "blobcam");
    } catch (IOException e) {
      throw new RuntimeException("Could not load ball model", e);
    }
    // Camera mount on the robot. MEASURE THESE (same meaning as in camera/BlobDetection).
    finder.cameraOffsetX = 7.0; // inches the lens sits FORWARD of robot center
    finder.cameraOffsetY = -2.0; // inches the lens sits LEFT of center (- = right)
    finder.cameraYawDegrees = 0.0; // camera direction vs robot forward (+ = left)

    // Returns null until Pedro is configured in pedro/Constants.create().
    Follower follower = Constants.create(hardwareMap);
    if (follower != null) follower.setPose(Pose.zero()); // put your real starting pose here

    try {
      waitForStart();
      // Read the robot pose right before detecting so both describe the same moment.
      if (follower != null) follower.update();
      Pose robot = follower != null ? follower.pose() : Pose.zero();

      Path path = finder.getPathToBall(robot); // runs the CNN once; null if no ball
      // To drive it: if (path != null) follower.follow(path); then call follower.update() in a loop.

      telemetry.addData("Robot pose", follower != null ? robot : "Pedro not set up; using (0, 0, 0)");
      telemetry.addData("Ball found", path != null);
      if (path != null) {
        telemetry.addData("Ball world X (in)", finder.ballX);
        telemetry.addData("Ball world Y (in)", finder.ballY);
        telemetry.addData("Path end", path.endPose());
      }
      telemetry.update();
      while (opModeIsActive()) sleep(50);
    } finally {
      finder.close();
    }
  }
}
