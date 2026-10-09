package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;

/** Camera firmware handles automatic exposure; this class adds no processing threads. */
public final class BiobuzzCameraExposure {
  public String status = "Waiting for camera", mode = "Unknown", supportedModes = "Unknown";
  public double milliseconds = -1;
  public boolean allowsFpsDrop = false;
  private long nextPoll;
  private Boolean appliedPriority;
  private ExposureControl control;

  // Poll at most once per second so USB control queries do not burden every video frame.
  public void update(
      VisionPortal portal, boolean automatic, double manualMs, boolean allowFpsDrop) {
    if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
    long now = System.nanoTime();
    if (now < nextPoll) return;
    nextPoll = now + 1_000_000_000L;
    try {
      if (control == null) {
        control = portal.getCameraControl(ExposureControl.class);
        if (control == null) {
          status = "Exposure control unavailable";
          return;
        }
        List<String> supported = new ArrayList<>();
        for (ExposureControl.Mode candidate : ExposureControl.Mode.values())
          if (control.isModeSupported(candidate)) supported.add(candidate.name());
        supportedModes = supported.toString();
      }
      ExposureControl.Mode desired = ExposureControl.Mode.Manual;
      if (automatic) {
        // Auto is a single-trigger mode in the SDK. These modes continuously adjust exposure.
        if (control.isModeSupported(ExposureControl.Mode.ContinuousAuto))
          desired = ExposureControl.Mode.ContinuousAuto;
        else if (control.isModeSupported(ExposureControl.Mode.AperturePriority))
          desired = ExposureControl.Mode.AperturePriority;
        else {
          status = "Continuous automatic exposure unsupported";
          readActual();
          return;
        }
      }
      if (!control.isModeSupported(desired)) {
        status = "Requested exposure mode unsupported";
        readActual();
        return;
      }
      if (control.getMode() != desired && !control.setMode(desired)) {
        status = "Camera rejected exposure mode";
        readActual();
        return;
      }
      status = automatic ? "Camera automatic exposure" : "Manual exposure";
      if (!automatic) {
        long min = control.getMinExposure(TimeUnit.MICROSECONDS);
        long max = control.getMaxExposure(TimeUnit.MICROSECONDS);
        if (!control.isExposureSupported() || min <= 0 || max < min || !Double.isFinite(manualMs))
          status = "Manual exposure duration unavailable/invalid";
        else {
          long requested = Math.max(min, Math.min(max, Math.round(manualMs * 1000)));
          if (Math.abs(control.getExposure(TimeUnit.MICROSECONDS) - requested) >= 100
              && !control.setExposure(requested, TimeUnit.MICROSECONDS))
            status = "Camera rejected exposure duration";
        }
      }
      // Disable the optional low-light FPS reduction by default, preserving real-time capture.
      if (appliedPriority == null || appliedPriority != allowFpsDrop) {
        boolean accepted = control.setAePriority(allowFpsDrop);
        appliedPriority = allowFpsDrop;
        if (!accepted) status += "; FPS-priority control unsupported";
      }
      readActual();
    } catch (RuntimeException error) {
      // Unsupported UVC controls should not kill an otherwise working vision stream.
      status = "Exposure control: " + error.getMessage();
    }
  }

  private void readActual() {
    mode = control.getMode().name();
    // UVC control readback is not sensor metadata for a captured frame. In our UC60 test,
    // auto mode kept returning the last manual duration even as brightness recovered.
    long micros = control.getExposure(TimeUnit.MICROSECONDS);
    milliseconds = micros > 0 ? micros / 1000. : -1;
    allowsFpsDrop = control.getAePriority();
  }
}
