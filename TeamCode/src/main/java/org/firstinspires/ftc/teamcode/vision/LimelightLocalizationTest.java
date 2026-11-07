package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Driver Station localization display. Works during INIT and PLAY without mapping any motors.
 * Edit LimelightLocalizationSettings for camera, pipeline and timing settings.
 */
@TeleOp(name = "BIOBUZZ Limelight Localization", group = "Vision")
public final class LimelightLocalizationTest extends LinearOpMode {
  private static final double METERS_PER_INCH = 0.0254;

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
        telemetry.addData("Localization", localization.getStatus());
        telemetry.addData("Mode", isStarted() ? "ACTIVE" : "INIT");
        showFieldPose(fix);
        showTags(localization);
        telemetry.update();
        sleep(loopDelayMs);
      }
    } catch (RuntimeException error) {
      showErrorUntilStopped(error.getMessage());
    }
  }

  /** Only display a current field fix; a lost target must not leave old coordinates on screen. */
  private void showFieldPose(LimelightLocalization.Estimate fix) {
    if (fix == null) {
      telemetry.addLine("No usable field position. No previous fix is displayed as current.");
      return;
    }
    telemetry.addData("Field XYZ (m)", "%.3f, %.3f, %.3f",
        fix.xMeters, fix.yMeters, fix.zMeters);
    telemetry.addData("Field XYZ (in)", "%.1f, %.1f, %.1f",
        fix.xMeters / METERS_PER_INCH, fix.yMeters / METERS_PER_INCH,
        fix.zMeters / METERS_PER_INCH);
    telemetry.addData("Roll / Pitch / Yaw (deg)", "%.1f / %.1f / %.1f",
        fix.rollDegrees, fix.pitchDegrees, fix.yawDegrees);
    telemetry.addData("Mapped tags", fix.mappedTags);
    telemetry.addData("Approximate age (ms)", "%.0f", fix.ageMs);
  }

  /** Camera-to-tag measurements help check visibility and tag-size setup independently. */
  private void showTags(LimelightLocalization localization) {
    for (LimelightLocalization.Tag tag : localization.getTags()) {
      telemetry.addData("Tag " + tag.id,
          "%s: range %.3f m; right/up/forward %.3f/%.3f/%.3f m",
          tag.cell, tag.distanceMeters, tag.rightMeters, tag.upMeters, tag.forwardMeters);
    }
  }

  /** Keep setup failures visible until STOP rather than immediately leaving the OpMode. */
  private void showErrorUntilStopped(String message) {
    telemetry.clearAll();
    telemetry.addData("Limelight setup/error", message);
    telemetry.update();
    while (!isStopRequested()) {
      sleep(100);
    }
  }
}
