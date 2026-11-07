package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

/** Camera-only bench test. Never maps, powers, or commands a motor/servo. */
@Config
@TeleOp(name = "BIOBUZZ Vision Test", group = "Vision")
public final class BiobuzzVisionTest extends LinearOpMode {
  // Live setting: higher confidence rejects more candidates but can miss difficult balls.
  public static double CONFIDENCE = .85;
  // Replace the nominal FOV with calibration for better distance estimates.
  public static double HORIZONTAL_FOV_DEGREES = 70;
  // Startup setting: INT8 reduces numeric precision while preserving the selected crop size.
  public static boolean USE_INT8 = true;
  // Startup crop size: INT8 supports 96/128; FP32 supports 96/128/160. Camera stays 640×480.
  public static int CNN_INPUT_SIZE = 128;
  // Startup setting: one runs every proposal and crop sequentially on the detector thread.
  public static int PIPELINE_WORKERS = 1;
  // Live setting: leave this worker idle between detections to reduce sustained Hub CPU use.
  // 60% is the initial measured-budget trial for about 50% TOTAL Hub CPU, not a hard CPU cap.
  public static double DETECTOR_DUTY_PERCENT = 60;
  // Full-resolution candidate search for the 640-wide camera; reducing this can harm accuracy.
  public static int PROPOSAL_WIDTH = 800;
  // Hardware configuration name, with a fallback only when exactly one webcam is attached.
  // Matches the "blobcam" name used by camera/BlobDetection in this project's robot config.
  public static String CAMERA_NAME = "blobcam";
  // Live camera settings. Automatic uses camera firmware; manual duration is clamped to UVC limits.
  public static boolean AUTO_EXPOSURE = true;
  public static boolean AE_ALLOW_FPS_DROP = false;
  public static double MANUAL_EXPOSURE_MS = 8;

  // Send the same telemetry values to the Driver Station and browser Dashboard.
  private void publish(FtcDashboard dashboard, Map<String, Object> data) {
    telemetry.clearAll();
    TelemetryPacket packet = new TelemetryPacket();
    for (Map.Entry<String, Object> entry : data.entrySet()) {
      telemetry.addData(entry.getKey(), entry.getValue());
      packet.put(entry.getKey(), entry.getValue());
    }
    telemetry.update();
    dashboard.sendTelemetryPacket(packet);
  }

  @Override
  public void runOpMode() {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    // Limit telemetry traffic independently of camera and detection rates.
    telemetry.setMsTransmissionInterval(150);
    VisionPortal portal = null;
    BiobuzzVisionProcessor processor = null;
    try {
      WebcamName webcam;
      try {
        webcam = hardwareMap.get(WebcamName.class, CAMERA_NAME);
        // Avoid silently choosing the wrong camera when several are connected.
      } catch (IllegalArgumentException missing) {
        List<WebcamName> cameras = ClassFactory.getInstance().getCameraManager().getAllWebcams();
        if (cameras.size() != 1)
          throw new IllegalStateException(
              "Configure '"
                  + CAMERA_NAME
                  + "' or connect exactly one USB webcam. Found "
                  + cameras.size());
        webcam = cameras.get(0);
      }
      // Load OpenCV before constructing the detector’s native image buffers.
      System.loadLibrary("opencv_java4");
      // Snapshot startup settings so telemetry describes this session even if config is edited.
      final boolean sessionInt8 = USE_INT8;
      final int sessionWorkers = Math.max(1, Math.min(3, PIPELINE_WORKERS));
      final int sessionSize = CNN_INPUT_SIZE;
      processor =
          new BiobuzzVisionProcessor(
              hardwareMap.appContext.getAssets(), sessionInt8, sessionWorkers, sessionSize);
      // MJPEG reduces USB bandwidth. The headless Hub does not need a local screen preview.
      portal =
          new VisionPortal.Builder()
              .setCamera(webcam)
              .setCameraResolution(new Size(640, 480))
              .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
              .enableLiveView(false)
              .addProcessor(processor)
              .build();
      // Control Hub has no local display. Dashboard and Driver Station
      // still receive the annotated processor image without that viewport.
      org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer.getInstance()
          .setSource(processor);
      dashboard.startCameraStream(processor, 10);
      BiobuzzCameraExposure exposure = new BiobuzzCameraExposure();
      // Works during INIT, including dashboard INIT without a Driver Station.
      while (!isStopRequested()) {
        exposure.update(portal, AUTO_EXPOSURE, MANUAL_EXPOSURE_MS, AE_ALLOW_FPS_DROP);
        // Copy live configuration into volatile worker settings without restarting the camera.
        processor.threshold = CONFIDENCE;
        processor.horizontalFov = HORIZONTAL_FOV_DEGREES;
        processor.proposalWidth = PROPOSAL_WIDTH;
        processor.dutyPercent = DETECTOR_DUTY_PERCENT;
        // Read one snapshot so every telemetry value belongs to the same completed detection.
        BiobuzzVisionProcessor.Result result = processor.latest;
        Map<String, Object> data = new LinkedHashMap<>();
        data.put("Mode", isStarted() ? "ACTIVE (vision only)" : "INIT (vision only)");
        data.put(
            "CNN precision",
            (sessionInt8 ? "INT8 / " : "FP32 / ") + sessionSize + "x" + sessionSize);
        data.put("Detector workers", sessionWorkers);
        data.put(
            "Detector duty budget %",
            Double.isFinite(DETECTOR_DUTY_PERCENT)
                ? Math.max(10, Math.min(100, DETECTOR_DUTY_PERCENT))
                : 100);
        data.put("Detector planned idle ms", processor.pacingIdleMs);
        data.put("Library threads", "TFLite 1 / OpenCV 1");
        data.put("Camera", portal.getCameraState().toString());
        data.put("Camera FPS", portal.getFps());
        data.put("Exposure control", exposure.status);
        data.put("Exposure mode", exposure.mode);
        data.put("Exposure ms", exposure.milliseconds);
        data.put("Exposure supported modes", exposure.supportedModes);
        data.put("AE allows FPS drop", exposure.allowsFpsDrop);
        data.put("Detection FPS", processor.detectionFps);
        // Skipping stale queued frames is intentional; it prevents growing latency.
        data.put("Skipped camera frames", processor.droppedFrames);
        data.put("Candidates", result.candidates);
        data.put("CNN calls", result.invocations);
        data.put(
            "CNN execution", sessionWorkers == 1 ? "Sequential crops" : "Parallel crop batches");
        // CPU sums can exceed elapsed time because multiple crops run concurrently.
        data.put("Proposals ms", result.proposalMs);
        data.put("Crop CPU sum ms", result.cropMs);
        data.put("CNN CPU sum ms", result.modelMs);
        data.put("Threshold", CONFIDENCE);
        data.put("Inference ms", result.milliseconds);
        data.put("Capture to result ms", result.captureToResultMs);
        data.put("Frame age ms", Double.isFinite(result.ageMs()) ? result.ageMs() : -1);
        data.put("Coordinates", "meters: X right, Y up, Z forward; approximate");
        if (!result.error.isEmpty()) data.put("Detector error", result.error);
        // Do not publish stale coordinates as current robot observations.
        boolean fresh = result.ageMs() < 1500 && result.error.isEmpty();
        data.put("Tracking", fresh ? "LIVE" : "NO FRESH DETECTIONS");
        data.put("Objects", fresh ? result.balls.size() : 0);
        if (fresh) {
          int i = 0;
          for (BiobuzzDetector.Ball ball : result.balls) {
            String key = "Ball " + (++i);
            data.put(
                key,
                String.format(
                    Locale.US, "%s %.1f%%", BiobuzzDetector.LABELS[ball.cls], 100 * ball.score));
            data.put(
                key + " XYZ",
                String.format(Locale.US, "(%+.3f, %+.3f, %.3f) m", ball.x, ball.y, ball.z));
            data.put(
                key + " angles",
                String.format(
                    Locale.US, "right %+.1f deg, up %+.1f deg", ball.azimuth, ball.elevation));
            data.put(key + " range", ball.range);
          }
        }
        // Yield between telemetry updates so this loop does not consume a CPU core.
        publish(dashboard, data);
        sleep(150);
      }
      // Keep setup errors visible in telemetry until the user stops the OpMode.
    } catch (Exception e) {
      while (!isStopRequested()) {
        Map<String, Object> data = new LinkedHashMap<>();
        data.put("Vision setup error", e.toString());
        data.put("Help", "Check webcam connection. Stop and INIT again after fixing.");
        // Yield between telemetry updates so this loop does not consume a CPU core.
        publish(dashboard, data);
        sleep(250);
      }
    } finally {
      // Stop camera callbacks before asking the background detector to shut down.
      dashboard.stopCameraStream();
      if (portal != null) portal.close();
      if (processor != null) processor.close();
    }
  }
}
