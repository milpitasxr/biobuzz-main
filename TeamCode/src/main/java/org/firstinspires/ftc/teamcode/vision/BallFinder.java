package org.firstinspires.ftc.teamcode.vision;

import static com.pedropathing.api.Paths.line;

import android.graphics.Canvas;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * On-demand ball detector. The camera streams, but the CNN runs only when you call detect().
 * Call detect() and read the results from one thread (your OpMode loop).
 */
public final class BallFinder implements AutoCloseable {
  // Results of the most recent detect() call, for the closest ball.
  // distanceInches is -1 when no ball was found.
  public double distanceInches = -1;
  public double angleDegrees = 0; // + = ball is to the right
  public String color = "None";
  // Every ball from the most recent detect() call, in case you need more than the closest one.
  // The other alliance's Nectar (set by Alliance.current) is already removed.
  public List<BiobuzzDetector.Ball> balls = Collections.emptyList();

  // Same meaning as the BIOBUZZ Vision Test settings.
  public double confidence = .85;
  public double horizontalFovDegrees = 70;

  // Camera mount on the robot, used by getPathToBall. MEASURE THESE on your robot.
  public double cameraOffsetX = 7.0; // inches the lens sits FORWARD of robot center
  public double cameraOffsetY = -2.0; // inches the lens sits LEFT of center (- = right)
  public double cameraYawDegrees = 0.0; // camera direction vs robot forward (+ = left)
  // Ball field position (inches) from the most recent successful getPathToBall call.
  public double ballX = 0, ballY = 0;

  private final BiobuzzDetector detector;
  private final VisionPortal portal;
  private final int previousOpenCvThreads;
  // The camera thread copies one frame into this buffer only when detect() asks for it.
  private final Object lock = new Object();
  private final Mat frame = new Mat();
  private boolean wanted, ready;

  public BallFinder(HardwareMap hardwareMap, String cameraName) throws IOException {
    System.loadLibrary("opencv_java4");
    previousOpenCvThreads = Core.getNumThreads();
    Core.setNumThreads(1);
    // INT8 model, one worker, 128x128 crops: the measured default configuration.
    detector = new BiobuzzDetector(hardwareMap.appContext.getAssets(), true, 1, 128);
    try {
      portal =
          new VisionPortal.Builder()
              .setCamera(hardwareMap.get(WebcamName.class, cameraName))
              .setCameraResolution(new Size(640, 480))
              .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
              .enableLiveView(false)
              .addProcessor(new FrameGrabber())
              .build();
    } catch (RuntimeException e) {
      detector.close();
      frame.release();
      Core.setNumThreads(previousOpenCvThreads);
      throw e;
    }
  }

  /** True once the camera is streaming and detect() can get a frame. */
  public boolean isCameraReady() {
    return portal.getCameraState() == VisionPortal.CameraState.STREAMING;
  }

  /**
   * Grabs the next camera frame and runs the CNN on it once. Blocks for roughly 0.2 s.
   *
   * @return true if a ball was found; the result variables are updated either way.
   */
  public boolean detect() {
    synchronized (lock) {
      ready = false;
      wanted = true;
      // Wait for the camera thread to hand over a fresh frame.
      long deadline = System.nanoTime() + 2_000_000_000L;
      while (!ready) {
        long remaining = deadline - System.nanoTime();
        if (remaining <= 0) {
          // Camera is not streaming yet.
          wanted = false;
          clearResults();
          return false;
        }
        try {
          lock.wait(remaining / 1_000_000L + 1);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          wanted = false;
          clearResults();
          return false;
        }
      }
    }
    // The camera thread writes the frame only while wanted is true, so reading it here is safe.
    // Drop the other alliance's Nectar. Pollen (yellow) is kept for both alliances.
    // Classes: 1 = Pollen yellow, 2 = Nectar red, 3 = Nectar blue.
    int opponentNectar = Alliance.current == Alliance.RED ? 3 : 2;
    List<BiobuzzDetector.Ball> ours = new ArrayList<>();
    for (BiobuzzDetector.Ball ball : detector.detect(frame, confidence, horizontalFovDegrees))
      if (ball.cls != opponentNectar) ours.add(ball);
    balls = Collections.unmodifiableList(ours);
    BiobuzzDetector.Ball closest = null;
    for (BiobuzzDetector.Ball ball : balls)
      if (closest == null || ball.range < closest.range) closest = ball;
    if (closest == null) {
      clearResults();
      return false;
    }
    distanceInches = closest.range * 39.37;
    angleDegrees = closest.azimuth;
    color = BiobuzzDetector.LABELS[closest.cls];
    return true;
  }

  /**
   * Runs the CNN once and returns the distance to the closest ball in inches, or -1 if no ball
   * was found. Blocks for roughly 0.2 s.
   */
  public double getDistance() {
    detect();
    return distanceInches;
  }

  /**
   * Runs the CNN once and returns a straight Pedro path from robotPose to the closest ball, or
   * null if no ball was found. Read robotPose right before calling this so both describe the
   * same moment. The path ends with the robot's center on the ball's center.
   */
  public Path getPathToBall(Pose robotPose) {
    double distance = getDistance();
    if (distance == -1) return null;
    double radians = Math.toRadians(angleDegrees); // Math.cos/sin need radians
    double forward = Math.cos(radians) * distance; // inches forward of the camera
    double left = -Math.sin(radians) * distance; // inches, + = left, - = right

    // Camera -> robot: rotate by the camera's yaw, then add the mount offset.
    // Robot frame: +x forward, +y left (same as Pedro).
    double yaw = Math.toRadians(cameraYawDegrees);
    double robotForward = cameraOffsetX + forward * Math.cos(yaw) - left * Math.sin(yaw);
    double robotLeft = cameraOffsetY + forward * Math.sin(yaw) + left * Math.cos(yaw);

    // Robot -> world: rotate by the robot's heading (radians), then add its field position.
    double heading = robotPose.heading();
    ballX = robotPose.x() + robotForward * Math.cos(heading) - robotLeft * Math.sin(heading);
    ballY = robotPose.y() + robotForward * Math.sin(heading) + robotLeft * Math.cos(heading);

    // Straight line to the ball, with the robot facing its direction of travel.
    return line(robotPose, new Pose(ballX, ballY)).tangent();
  }

  private void clearResults() {
    distanceInches = -1;
    angleDegrees = 0;
    color = "None";
    balls = Collections.emptyList();
  }

  @Override
  public void close() {
    // Stop camera callbacks before freeing the frame buffer and model.
    portal.close();
    detector.close();
    frame.release();
    Core.setNumThreads(previousOpenCvThreads);
  }

  // Runs on the camera thread for every frame, but only copies when a detection was requested.
  private final class FrameGrabber implements VisionProcessor {
    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
      synchronized (lock) {
        if (wanted) {
          if (input.channels() == 4) Imgproc.cvtColor(input, frame, Imgproc.COLOR_RGBA2RGB);
          else input.copyTo(frame);
          wanted = false;
          ready = true;
          lock.notifyAll();
        }
      }
      return null;
    }

    @Override
    public void onDrawFrame(
        Canvas canvas, int width, int height, float scale, float density, Object context) {}
  }
}
