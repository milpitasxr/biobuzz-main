package org.firstinspires.ftc.teamcode.vision;

/** Hardware-independent checks. A null rejection means the supplied data passed validation. */
public final class LimelightPoseQuality {
  private LimelightPoseQuality() {}

  /**
   * Checks a frame before either field localization or camera-relative tag diagnostics use it.
   * All ages/latencies are milliseconds. unchangedMs measures time since the camera timestamp
   * last changed; staleMs measures the SDK's data age. Both matter for a frozen stream.
   */
  public static String frameRejection(boolean valid, int actualPipeline, int expectedPipeline,
      long staleMs, double unchangedMs, double maxAgeMs, double latencyMs) {
    if (!Double.isFinite(maxAgeMs) || maxAgeMs <= 0) {
      return "Invalid maximum result age";
    }
    if (actualPipeline != expectedPipeline) {
      return "Waiting for selected pipeline";
    }
    if (staleMs < 0 || !Double.isFinite(unchangedMs) || unchangedMs < 0
        || staleMs > maxAgeMs || unchangedMs > maxAgeMs) {
      return "Stale/frozen Limelight result";
    }
    if (!valid) {
      return "No visible target";
    }
    // Use the older age estimate rather than adding two overlapping age measurements.
    if (!Double.isFinite(latencyMs) || latencyMs < 0
        || latencyMs + Math.max(staleMs, unchangedMs) > maxAgeMs) {
      return "Pose too old";
    }
    return null;
  }

  /**
   * Adds field-pose checks to the shared frame checks. pose must contain six finite values:
   * X/Y/Z in meters followed by roll/pitch/yaw in degrees. mappedTags counts tags in the solve,
   * not merely visible tags. Unknown tags cannot establish a field position.
   */
  public static String rejection(boolean valid, int actualPipeline, int expectedPipeline,
      long staleMs, double unchangedMs, double maxAgeMs, int mappedTags,
      double latencyMs, double[] pose) {
    String frameProblem = frameRejection(valid, actualPipeline, expectedPipeline,
        staleMs, unchangedMs, maxAgeMs, latencyMs);
    if (frameProblem != null) {
      return frameProblem;
    }
    if (mappedTags < 1) {
      return "No mapped tags: check field map and Full 3D";
    }
    if (pose == null || pose.length != 6) {
      return "No robot field pose";
    }
    for (double value : pose) {
      if (!Double.isFinite(value)) {
        return "Invalid robot field pose";
      }
    }
    // Zero is a valid field coordinate. Mapped tag count distinguishes a missing solve.
    return null;
  }
}
