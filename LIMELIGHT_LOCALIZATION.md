# Limelight AprilTag localization

Java code: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/`.
Driver Station OpMode: **BIOBUZZ Limelight Localization** (Vision).

This adds MegaTag1 robot field position (X/Y/Z in meters) and roll/pitch/yaw
(degrees), plus the Python app's individual-tag camera-space ranges and Hive
labels. Limelight performs detection and the field pose solve. This uses the
FTC SDK Limelight API; no Python runtime or new dependency is required.

## Editable settings

All team-editable values are in `LimelightLocalizationSettings.java`. Comments
explain the units and purpose of each value. Edit, rebuild/install, then re-INIT.

| Setting | Default | Purpose |
| --- | --- | --- |
| `CAMERA_NAME` | `limelight` | Name in the robot hardware configuration |
| `PIPELINE` | `0` | AprilTag pipeline slot, 0-9 |
| `FIELD_MAP_AND_MOUNT_VERIFIED` | `false` | Enable field fixes after physical setup |
| `POLL_RATE_HZ` | `50` | Result polling rate, 1-100 Hz |
| `MAX_RESULT_AGE_MS` | `250` | Freshness limit, including reported processing latency |
| `TELEMETRY_INTERVAL_MS` | `100` | Driver Station transmission interval |
| `LOOP_DELAY_MS` | `20` | Pause between test-mode updates |

The reader copies its settings at construction. Other OpModes can use its
explicit constructor to supply polling and maximum-age values independently.
The test mode uses the repository's existing Panels telemetry for browser readings;
settings are changed in source.

## Code guide

- `LimelightLocalizationSettings`: configuration and units.
- `LimelightLocalization`: camera lifecycle, freshness tracking and result snapshots.
- `LimelightPoseQuality`: hardware-independent rejection rules.
- `LimelightLocalizationTest`: readable Driver Station telemetry during INIT/PLAY.
- `tools/LimelightPoseQualityCheck.java`: standalone regression checks.

## Setup before field coordinates are enabled

1. Connect the Limelight to the Control Hub and add it to the active robot
   configuration as `limelight` (or change `CAMERA_NAME`).
2. In the Limelight web interface, choose a tag36h11 AprilTag pipeline and enable
   Full 3D (turn off Skip 3D). Set `PIPELINE` in `LimelightLocalizationSettings` to that
   pipeline number (0 is just the default). Re-INIT after changing startup settings.
3. Load the correct BIOBUZZ `.fmap` containing the actual fixed tag positions,
   orientations and sizes. For a practice setup, use a map of the actual printed
   tag placements. A printout placed arbitrarily in the room is not a field map.
   Do not use the SDK 11.2.1 season library as a substitute for a BIOBUZZ map.
4. Set the actual tag size in the Limelight. The Python project records 82.55 mm
   (3.25 in) for IDs 30-45; verify against the current manual and your physical
   tags. The old Python UI could compensate for a 101.6 mm pipeline setting.
   This Java reader intentionally does not scale the final field coordinates:
   configure the pipeline/map correctly instead. Until then, displayed ranges
   can also be wrong.
5. Enter the measured camera position and angles in Limelight's robot-space
   configuration: Forward/Right/Up in meters, Roll/Pitch/Yaw in degrees. These
   describe the lens relative to your chosen robot origin, including its tilt
   toward overhead tags. Use the web interface's 3D view to verify the mounting.
6. After verifying these settings, set `FIELD_MAP_AND_MOUNT_VERIFIED = true` in `LimelightLocalizationSettings`, rebuild/install the app, and
   re-INIT. This is a manual setup acknowledgment, not automatic calibration or
   map validation. Until then, the reader reports SETUP REQUIRED and returns no
   field fix; individual tag diagnostics remain available.

## Browser distance display

With the Control Hub connected over USB, forward Hub ports 8001 and 8002 using ADB.
Open http://localhost:8001 and add the OpMode and Telemetry panels. Select
**BIOBUZZ Limelight Localization** and press INIT. The Telemetry panel shows visible
IDs, camera-to-tag distance in meters/inches, and right/up/forward offsets.
These distance readings work with the field-position setup flag still false;
accurate ranges still require the correct physical tag size and Full 3D.

## On-robot check

Select the localization OpMode. It displays results during INIT and PLAY and
does not initialize motors. Check X/Y/Z and yaw at several measured positions,
then turn the robot and verify the position stays approximately constant.
Cover the tags or disconnect the camera: field coordinates should disappear
and a diagnostic should replace them. STOP releases Limelight polling.

The returned coordinate frame is the loaded Limelight map's frame. A standard
FTC map uses the field-center origin; a custom map may differ. Do not feed these
coordinates straight into Pedro Pathing without explicitly converting its
origin, axes, distance units and heading convention.

## Using the reader from another OpMode

Construct `LimelightLocalization(hardwareMap, "limelight", pipeline, true)` after
completing setup, call `start()` once, and call `update()` in each loop. A non-null
`Estimate` is the current usable fix. A null result means localization is
unavailable; it does not mean the robot is at zero. Call `close()` in `stop()` or
`finally`. `getTags()` provides individual tag offsets/ranges; these describe the
camera-to-tag relationship, not the robot's field position.

The reader rejects wrong-pipeline results, missing mapped tags, nonfinite poses,
and results beyond `MAX_RESULT_AGE_MS` (250 ms by default), including reported
processing latency. Tag diagnostics use the same frame-freshness checks. Repeated reads of a still-current frame can return the same fix; compare
`frameTimestamp` before applying corrections to an odometry estimator. This
timestamp is Limelight-local, not Control Hub time. `ageMs` is approximate and
is not a synchronized capture timestamp for delayed odometry fusion.

MegaTag1 needs no external heading. MegaTag2 is not enabled because it requires
a correctly field-aligned heading source. This reader does not provide odometry
between sightings, automatically move the robot, or change existing drive code.
Coplanar/single-tag observations can be ambiguous; validate the pose before
using it for autonomous motion. This is not an uncertainty-aware fusion filter.

## Validation

From the repository root, run the hardware-independent checks with:

```powershell
New-Item -ItemType Directory -Force build/localization-checks | Out-Null
javac -d build/localization-checks TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/LimelightPoseQuality.java tools/LimelightPoseQualityCheck.java
java -cp build/localization-checks LimelightPoseQualityCheck
```

`tools/LimelightPoseQualityCheck.java` is a standalone Java regression harness for
origin poses, tag loss, wrong pipeline, stale/frozen frames, bad values and age
limits. Build the app with `gradlew.bat :TeamCode:assembleDebug`.
Live accuracy must be checked after the map and camera mounting are configured.

References:
- https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
- https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization

The Hive ID labels were ported from `robotics/tagrange/tags.py`; they are display
labels only. Localization uses the uploaded field map, not these labels.
