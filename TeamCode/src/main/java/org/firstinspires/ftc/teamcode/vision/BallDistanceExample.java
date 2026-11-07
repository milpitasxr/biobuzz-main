package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.IOException;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Debug OpMode: press gamepad1 A to rerun finder.getPathToBall() and show the result. */
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
      // Debug loop: works in INIT and after START. Press gamepad1 A to rerun the CNN.
      Path path = null;
      Pose robot = Pose.zero();
      int runs = 0;
      double runMs = 0;
      boolean wasPressed = false;
      while (opModeInInit() || opModeIsActive()) {
        if (follower != null) follower.update();

        if (gamepad1.a && !wasPressed) {
          // Read the robot pose right before detecting so both describe the same moment.
          robot = follower != null ? follower.pose() : Pose.zero();
          long start = System.nanoTime();
          path = finder.getPathToBall(robot); // runs the CNN once; null if no ball
          runMs = (System.nanoTime() - start) / 1e6;
          runs++;
          // To drive it: if (path != null) follower.follow(path); then keep calling follower.update().
        }
        wasPressed = gamepad1.a;

        telemetry.addData("Press A", "to rerun the CNN");
        telemetry.addData("Camera ready", finder.isCameraReady());
        telemetry.addData("Alliance", Alliance.current);
        telemetry.addData("Runs", runs);
        telemetry.addData("Last run (ms)", "%.0f", runMs);
        telemetry.addData(
            "Robot pose", follower != null ? robot : "Pedro not set up; using (0, 0, 0)");
        telemetry.addData("Ball found", path != null);
        if (path != null) {
          telemetry.addData("Color", finder.color);
          telemetry.addData("Distance (in)", "%.1f", finder.distanceInches);
          telemetry.addData("Angle (deg)", "%.1f", finder.angleDegrees);
          telemetry.addData("Ball world X (in)", "%.1f", finder.ballX);
          telemetry.addData("Ball world Y (in)", "%.1f", finder.ballY);
          telemetry.addData("Path end", path.endPose());
        }
        telemetry.addData("Balls seen (after alliance filter)", finder.balls.size());
        telemetry.update();
        sleep(20);
      }
    } finally {
      finder.close();
    }
  }
}
