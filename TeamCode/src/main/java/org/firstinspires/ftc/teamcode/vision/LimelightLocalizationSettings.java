package org.firstinspires.ftc.teamcode.vision;

/**
 * Edit this file to configure the localization test without changing the detection code.
 *
 * <p>Settings are read at INIT. After editing, rebuild/install the app and restart the OpMode.
 * The field map, physical tag size and camera mounting measurements belong in the Limelight
 * web interface; see LIMELIGHT_LOCALIZATION.md for that setup.
 */
public final class LimelightLocalizationSettings {
  private LimelightLocalizationSettings() {}

  /** Must match the Limelight name in the Driver Station's active robot configuration. */
  public static String CAMERA_NAME = "limelight";

  /** AprilTag pipeline slot on the Limelight, from 0 through 9. Enable Full 3D in this slot. */
  public static int PIPELINE = 0;

  /**
   * Set true after verifying the field map, tag sizes and camera mount in the web interface.
   * False allows tag diagnostics but prevents unconfigured field coordinates being used.
   */
  public static boolean FIELD_MAP_AND_MOUNT_VERIFIED = false;

  /** How often the Control Hub requests results. This does not change the camera's frame rate. */
  public static int POLL_RATE_HZ = 50;

  /** Maximum acceptable result age in milliseconds, including reported processing latency. */
  public static double MAX_RESULT_AGE_MS = 250;

  /** Minimum time between Driver Station telemetry transmissions, in milliseconds. */
  public static int TELEMETRY_INTERVAL_MS = 100;

  /** Pause between test-mode updates, in milliseconds; 20 ms gives roughly 50 updates/second. */
  public static int LOOP_DELAY_MS = 20;
}
