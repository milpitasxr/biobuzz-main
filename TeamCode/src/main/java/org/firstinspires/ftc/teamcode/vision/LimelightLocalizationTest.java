package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Driver Station localization display. Works during INIT and PLAY without mapping any motors.
 * Edit LimelightLocalizationSettings for camera, pipeline and timing settings.
 */
@TeleOp(name = "BIOBUZZ Limelight Localization", group = "Vision")
public final class LimelightLocalizationTest extends LinearOpMode {
  private static final double METERS_PER_INCH = 0.0254;
  // The same readings are published to Panels in the browser and to the Driver Station.
  private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

  @Override
  public void runOpMode() {
    // Snapshot timing at INIT so one session uses consistent settings.
    final int telemetryIntervalMs = LimelightLocalizationSettings.TELEMETRY_INTERVAL_MS;
    final int loopDelayMs = LimelightLocalizationSettings.LOOP_DELAY_MS;
    if (telemetryIntervalMs <= 0 || loopDelayMs <= 0) {
      showErrorUntilStopped("Telemetry interval and loop delay must be positive milliseconds");
      return;
    }
    telemetry.setMsTransmissionInterval(telemetryIntervalMs);
    panels.setUpdateInterval(telemetryIntervalMs);

    // The reader is closed automatically on STOP or an exception, releasing background polling.
    try (LimelightLocalization localization = new LimelightLocalization(
        hardwareMap,
        LimelightLocalizationSettings.CAMERA_NAME,
        LimelightLocalizationSettings.PIPELINE,
        LimelightLocalizationSettings.FIELD_MAP_AND_MOUNT_VERIFIED)) {
      localization.start();

      // Read during INIT too, so setup can be checked before pressing PLAY.
      while (!isStopRequested()) {
        LimelightLocalization.Estimate fix = localization.update();
        addReading("Localization", localization.getStatus());
        addReading("Mode", isStarted() ? "ACTIVE" : "INIT");
        showFieldPose(fix);
        showTags(localization);
        panels.update(telemetry);
        sleep(loopDelayMs);
      }
    } catch (RuntimeException error) {
      showErrorUntilStopped(error.getMessage());
    }
  }

  /** Only display a current field fix; a lost target must not leave old coordinates on screen. */
  private void showFieldPose(LimelightLocalization.Estimate fix) {
    if (fix == null) {
      panels.addLine("No usable field position. No previous fix is displayed as current.");
      return;
    }
    addReading("Field XYZ (m)", "%.3f, %.3f, %.3f",
        fix.xMeters, fix.yMeters, fix.zMeters);
    addReading("Field XYZ (in)", "%.1f, %.1f, %.1f",
        fix.xMeters / METERS_PER_INCH, fix.yMeters / METERS_PER_INCH,
        fix.zMeters / METERS_PER_INCH);
    addReading("Roll / Pitch / Yaw (deg)", "%.1f / %.1f / %.1f",
        fix.rollDegrees, fix.pitchDegrees, fix.yawDegrees);
    addReading("Mapped tags", fix.mappedTags);
    addReading("Approximate age (ms)", "%.0f", fix.ageMs);
  }

  /** Camera-to-tag measurements help check visibility and tag-size setup independently. */
  private void showTags(LimelightLocalization localization) {
    addReading("Visible tags with distance", localization.getTags().size());
    if (localization.getTags().isEmpty()) {
      panels.addLine("Aim at an AprilTag; enable Full 3D and check the pipeline/tag size.");
    }
    for (LimelightLocalization.Tag tag : localization.getTags()) {
      addReading("Tag " + tag.id,
          "%s: range %.3f m; right/up/forward %.3f/%.3f/%.3f m",
          tag.cell, tag.distanceMeters, tag.rightMeters, tag.upMeters, tag.forwardMeters);
      addReading("Tag " + tag.id + " distance (in)", "%.1f",
          tag.distanceMeters / METERS_PER_INCH);
    }
  }

  /** Plain readings and formatted readings share one destination for both displays. */
  private void addReading(String label, Object value) {
    panels.addData(label, value);
  }

  private void addReading(String label, String format, Object... values) {
    panels.addData(label, String.format(Locale.US, format, values));
  }

  /** Keep setup failures visible until STOP rather than immediately leaving the OpMode. */
  private void showErrorUntilStopped(String message) {
    telemetry.clearAll();
    panels.getLines().clear();
    while (!isStopRequested()) {
      addReading("Limelight setup/error", message);
      panels.update(telemetry);
      sleep(100);
    }
  }
}
