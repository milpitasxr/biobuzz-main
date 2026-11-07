package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Reads MegaTag1 robot localization from a Limelight. The camera solves the pose;
 * this class checks its freshness and exposes values in meters and degrees.
 *
 * <p>Call start() once, update() in each robot loop, and close() when the OpMode stops.
 * Use this reader from one OpMode thread. It does not drive motors or update odometry.
 */
public final class LimelightLocalization implements AutoCloseable {
  /** One accepted robot pose in the uploaded field map's coordinate system. */
  public static final class Estimate {
    /** Robot position in meters; the origin and axes come from the uploaded field map. */
    public final double xMeters, yMeters, zMeters;
    /** Robot orientation in degrees. Yaw is the heading about the map's vertical axis. */
    public final double rollDegrees, pitchDegrees, yawDegrees;
    /** Limelight-local frame timestamp. Compare for duplicates; do not compare to Hub time. */
    public final double frameTimestamp;
    /** Approximate age including reported processing time, in milliseconds. */
    public final double ageMs;
    /** Number of mapped tags contributing to the field pose. */
    public final int mappedTags;

    private Estimate(double[] pose, LLResult result, double ageMs) {
      xMeters = pose[0];
      yMeters = pose[1];
      zMeters = pose[2];
      rollDegrees = pose[3];
      pitchDegrees = pose[4];
      yawDegrees = pose[5];
      frameTimestamp = result.getTimestamp();
      mappedTags = result.getBotposeTagCount();
      this.ageMs = ageMs;
    }
  }

  /** Individual tag range, measured from the camera lens rather than the field origin. */
  public static final class Tag {
    /** AprilTag ID and the display label inherited from the Python range viewer. */
    public final int id;
    public final String cell;
    /** Camera-relative offsets in meters: right, up and out through the lens. */
    public final double rightMeters, upMeters, forwardMeters;
    /** Straight-line distance from the camera lens to the tag center, in meters. */
    public final double distanceMeters;

    private Tag(int id, Position cameraPosition) {
      this.id = id;
      cell = cellFor(id);
      rightMeters = cameraPosition.x;
      // Limelight camera-space Y points down; the display uses positive up.
      upMeters = -cameraPosition.y;
      forwardMeters = cameraPosition.z;
      distanceMeters = Math.sqrt(
          rightMeters * rightMeters + upMeters * upMeters + forwardMeters * forwardMeters);
    }
  }

  private final Limelight3A camera;
  private final int pipeline;
  private final boolean setupVerified;
  private final int pollRateHz;
  private final double maxResultAgeMs;

  // Track changes in the camera's timestamp, even if polling keeps returning a frozen frame.
  private double previousTimestamp = Double.NaN;
  private long frameChangedAtNanos;
  private String status = "Waiting for Limelight";
  private List<Tag> tags = Collections.emptyList();

  /** Uses the polling/freshness settings from LimelightLocalizationSettings at construction. */
  public LimelightLocalization(HardwareMap hardwareMap, String name, int pipeline,
      boolean setupVerified) {
    this(hardwareMap, name, pipeline, setupVerified,
        LimelightLocalizationSettings.POLL_RATE_HZ,
        LimelightLocalizationSettings.MAX_RESULT_AGE_MS);
  }

  /** Explicit settings overload for other OpModes; values remain fixed for this reader. */
  public LimelightLocalization(HardwareMap hardwareMap, String name, int pipeline,
      boolean setupVerified, int pollRateHz, double maxResultAgeMs) {
    if (name == null || name.trim().isEmpty()) {
      throw new IllegalArgumentException("Camera name must not be blank");
    }
    if (pipeline < 0 || pipeline > 9) {
      throw new IllegalArgumentException("Pipeline must be 0-9");
    }
    if (pollRateHz < 1 || pollRateHz > 100) {
      throw new IllegalArgumentException("Poll rate must be 1-100 Hz");
    }
    if (!Double.isFinite(maxResultAgeMs) || maxResultAgeMs <= 0) {
      throw new IllegalArgumentException("Maximum result age must be positive and finite");
    }
    this.pipeline = pipeline;
    this.setupVerified = setupVerified;
    this.pollRateHz = pollRateHz;
    this.maxResultAgeMs = maxResultAgeMs;
    camera = hardwareMap.get(Limelight3A.class, name);
  }

  /** Selects the configured pipeline and starts polling. Call once before reading results. */
  public void start() {
    previousTimestamp = Double.NaN;
    camera.setPollRateHz(pollRateHz);
    if (!camera.pipelineSwitch(pipeline)) {
      throw new IllegalStateException("Cannot select Limelight pipeline");
    }
    camera.start();
  }

  /**
   * Returns a current usable fix, or null when setup, visibility or quality checks fail.
   * A missing fix is never replaced with the last pose or with (0, 0, 0).
   */
  public Estimate update() {
    tags = Collections.emptyList();
    LLResult result = camera.getLatestResult();
    if (result == null) {
      return unavailable("Waiting for Limelight connection/data");
    }

    double timestamp = result.getTimestamp();
    if (!Double.isFinite(timestamp)) {
      return unavailable("Invalid frame timestamp");
    }
    long nowNanos = System.nanoTime();
    if (timestamp != previousTimestamp) {
      previousTimestamp = timestamp;
      frameChangedAtNanos = nowNanos;
    }
    double unchangedMs = (nowNanos - frameChangedAtNanos) / 1_000_000.0;
    long staleMs = result.getStaleness();
    double latencyMs = result.getCaptureLatency()
        + result.getTargetingLatency() + result.getParseLatency();

    // Tag diagnostics work before the field map/mount are verified, but still require fresh data.
    String frameProblem = LimelightPoseQuality.frameRejection(
        result.isValid(), result.getPipelineIndex(), pipeline,
        staleMs, unchangedMs, maxResultAgeMs, latencyMs);
    if (frameProblem != null) {
      return unavailable(frameProblem);
    }
    tags = readTags(result);
    if (!setupVerified) {
      return unavailable("SETUP REQUIRED: verify field map, tag size and camera mount");
    }

    double[] pose = readFieldPose(result.getBotpose());
    String rejection = LimelightPoseQuality.rejection(
        result.isValid(), result.getPipelineIndex(), pipeline, staleMs, unchangedMs,
        maxResultAgeMs, result.getBotposeTagCount(), latencyMs, pose);
    if (rejection != null) {
      return unavailable(rejection);
    }

    status = "TRACKING (MegaTag1)";
    return new Estimate(pose, result, latencyMs + Math.max(staleMs, unchangedMs));
  }

  /** Converts SDK pose objects into the ordered XYZ/roll/pitch/yaw values used by validation. */
  private static double[] readFieldPose(Pose3D fieldPose) {
    if (fieldPose == null) {
      return null;
    }
    Position position = fieldPose.getPosition().toUnit(DistanceUnit.METER);
    return new double[] {
        position.x, position.y, position.z,
        fieldPose.getOrientation().getRoll(AngleUnit.DEGREES),
        fieldPose.getOrientation().getPitch(AngleUnit.DEGREES),
        fieldPose.getOrientation().getYaw(AngleUnit.DEGREES)
    };
  }

  /** Copies only finite, forward-facing camera-space tag poses into a read-only snapshot. */
  private static List<Tag> readTags(LLResult result) {
    List<Tag> visibleTags = new ArrayList<>();
    List<LLResultTypes.FiducialResult> detections = result.getFiducialResults();
    if (detections == null) {
      return Collections.emptyList();
    }
    for (LLResultTypes.FiducialResult detection : detections) {
      Pose3D targetPose = detection.getTargetPoseCameraSpace();
      if (targetPose == null) {
        continue;
      }
      Position position = targetPose.getPosition().toUnit(DistanceUnit.METER);
      if (Double.isFinite(position.x) && Double.isFinite(position.y)
          && Double.isFinite(position.z) && position.z > 0) {
        visibleTags.add(new Tag(detection.getFiducialId(), position));
      }
    }
    return Collections.unmodifiableList(visibleTags);
  }

  /** Records why this update did not produce a usable field fix. */
  private Estimate unavailable(String reason) {
    status = reason;
    return null;
  }

  /** Human-readable status from the most recent update. */
  public String getStatus() {
    return status;
  }

  /** Current tag diagnostics; empty when the latest frame is missing, invalid or stale. */
  public List<Tag> getTags() {
    return tags;
  }

  /** Python viewer labels only. The uploaded field map supplies localization coordinates. */
  public static String cellFor(int id) {
    if (id >= 30 && id <= 33) {
      return "Red opposite audience";
    }
    if (id >= 34 && id <= 37) {
      return "Red audience";
    }
    if (id >= 38 && id <= 41) {
      return "Blue audience";
    }
    if (id >= 42 && id <= 45) {
      return "Blue opposite audience";
    }
    return "Other tag";
  }

  /** Stops background polling; try-with-resources also calls this when an error occurs. */
  @Override
  public void close() {
    camera.stop();
  }
}
