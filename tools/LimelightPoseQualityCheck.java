import org.firstinspires.ftc.teamcode.vision.LimelightPoseQuality;

/** Standalone regression checks; run with javac/java, without a connected robot. */
public final class LimelightPoseQualityCheck {
  private static int checks;

  // Each case states whether a field fix should be accepted, followed by simulated input.
  private static void check(boolean shouldAccept, boolean valid, int pipeline, long staleMs,
      double unchangedMs, int mappedTags, double latencyMs, double[] pose) {
    String reason = LimelightPoseQuality.rejection(valid, pipeline, 0, staleMs, unchangedMs,
        250, mappedTags, latencyMs, pose);
    if ((reason == null) != shouldAccept) {
      throw new AssertionError("Case " + (checks + 1) + " failed: " + reason);
    }
    checks++;
  }

  /** Covers accepted poses plus each class of invalid input without requiring hardware. */
  public static void main(String[] args) {
    // A zero coordinate is legitimate; so are negative positions and headings.
    double[] origin = {0, 0, 0, 0, 0, 0};
    check(true, true, 0, 10, 10, 2, 20, origin);
    check(true, true, 0, 0, 0, 1, 0, new double[] {-1, 2, 0, -5, 8, -179});
    // Visibility, pipeline, freshness and mapped-tag failures must suppress the fix.
    check(false, false, 0, 0, 0, 2, 0, origin);
    check(false, true, 1, 0, 0, 2, 0, origin);
    check(false, true, 0, 251, 0, 2, 0, origin);
    check(false, true, 0, 0, 251, 2, 0, origin);
    check(false, true, 0, 0, 0, 0, 0, origin);
    check(false, true, 0, 0, 0, 2, 0, null);
    // Nonfinite values must never reach driving or odometry code.
    check(false, true, 0, 0, 0, 2, 0, new double[] {Double.NaN, 0, 0, 0, 0, 0});
    check(false, true, 0, 0, 0, 2, 0, new double[] {0, 0, 0, 0, 0, Double.POSITIVE_INFINITY});
    // Processing latency also counts toward the maximum permitted age.
    check(false, true, 0, 100, 10, 2, 151, origin);
    check(false, true, 0, 0, 200, 2, 51, origin);
    check(false, true, 0, 0, 0, 2, Double.NaN, origin);
    check(false, true, 0, -1, 0, 2, 0, origin);
    check(false, true, 0, 0, Double.NaN, 2, 0, origin);
    check(false, true, 0, 0, -1, 2, 0, origin);
    // Both field and tag diagnostics share these configuration checks.
    if (LimelightPoseQuality.frameRejection(true, 0, 0, 0, 0, Double.NaN, 0) == null
        || LimelightPoseQuality.frameRejection(true, 0, 0, 0, 0, 0, 0) == null) {
      throw new AssertionError("Invalid age settings were accepted");
    }
    checks += 2;
    System.out.println("Passed " + checks + " localization quality checks.");
  }
}
