package org.firstinspires.ftc.teamcode.camera;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Finds Pollen (yellow) and Nectar (alliance red/blue) balls and reports where they are
 * relative to the robot.
 *
 * How distance works now:
 *   1. FLOOR METHOD (main answer): the balls sit on the floor, and we know how high the camera is
 *      and how far it tilts down. So the ball's HEIGHT in the image tells us how far away it is.
 *      This doesn't care how big the blob looks, so a half-detected ball still gives a good distance.
 *   2. SIZE METHOD (sanity check): the old "known diameter / apparent radius" formula.
 *   If the two methods disagree a lot, the blob is probably junk (a merged blob, a piece of a ball,
 *   or something that isn't a ball at all), so we throw it out. This kills a lot of false positives.
 */
@TeleOp(name = "Blob detection", group = "vision")
public class BlobDetection extends LinearOpMode {

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // CAMERA BASICS
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    private static final String CAMERA_NAME = "blobcam"; // must match the name in the robot config exactly
    private static final int FRAME_WIDTH  = 640;          // image width in pixels
    private static final int FRAME_HEIGHT = 480;          // image height in pixels

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // FOCAL LENGTH (turns pixels into angles). CALIBRATE THIS.
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // The spec-sheet FOV is only a starting guess. Spec FOVs are often the DIAGONAL angle or rounded,
    // and using the wrong one makes every distance wrong by the same percentage.
    private static final double FOV_DEG = 76.0;

    // After calibrating (see the "Focal if @" telemetry line), put the real value here.
    // Leave it at 0 to keep using the FOV guess.
    private static final double FOCAL_PX_CALIBRATED = 0.0;

    // Use the calibrated number if we have one, otherwise estimate from the FOV.
    private static final double FOCAL_PX = (FOCAL_PX_CALIBRATED > 0)
            ? FOCAL_PX_CALIBRATED
            : (FRAME_WIDTH / 2.0) / Math.tan(Math.toRadians(FOV_DEG / 2.0));

    // Calibration helper: put a ball straight in front of the lens at exactly this straight-line
    // distance (lens to ball center). Telemetry then shows the focal length that would make the
    // size formula correct. Do it at a few distances (12, 24, 36 in) and average the numbers.
    private static final double CALIB_DIST_IN = 24.0;

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // CAMERA IMAGE SETTINGS. EDIT IN COMP
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    private static final int EXPOS_MS = 6;          // shorter = darker but crisper (less motion blur / rolling-shutter smear)
    private static final int GAIN = 100;            // brightens the image but adds noise; use it to make up for a short exposure
    private static final int WHITE_BALANCE = 4000;  // color temperature of "white"; change it if the ball colors look off in the stream
    private static final double FOCUS_VAL = 0;      // fixed focus so the image doesn't pulse while autofocus hunts

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // CAMERA MOUNT. MUST BE MEASURED ON THE ROBOT
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    private static final double CAM_OFFSET_X = 7.0;   // inches the lens sits FORWARD of robot center
    private static final double CAM_OFFSET_Y = -2.0;  // inches the lens sits LEFT of robot center (negative = right)
    private static final double CAM_YAW_DEG = 0.0;    // where the camera points relative to robot forward. NEG IF RIGHT
    private static final double CAM_HEIGHT_IN = 8.0;  // lens height above the floor. MEASURE IT
    private static final double CAM_PITCH_DEG = 15.0; // how far the camera tilts DOWN from level. Use a phone level app. MEASURE IT
    // The floor method needs the camera ABOVE the ball's center and tilted down (or level).
    // A couple of degrees of pitch error is the biggest source of distance error, so measure carefully.

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // GAME PIECES
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    private static final double POLLEN_DIAM_IN = 2.8;
    private static final double NECTAR_DIAM_IN = 3.6;

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // COLOR RANGES. TUNE THESE UNDER YOUR LIGHTING
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // The built-in ColorRange.YELLOW/RED/BLUE are wide on purpose, which is why they catch tape,
    // bumpers and shirts. These are tighter starting points. Tune them by watching the Camera Stream:
    // narrow the range until only the ball gets outlined, then check the things that used to false-trigger.

    // Pollen: HSV (OpenCV scale: H 0-180, S 0-255, V 0-255).
    // H = which color (yellow is roughly 20-35), S = how strong the color is (raise the min to reject
    // washed-out stuff), V = brightness (raise the min to reject dark shadows).
    private static final ColorRange POLLEN_RANGE = new ColorRange(
            ColorSpace.HSV,
            new Scalar(18, 120, 90),
            new Scalar(35, 255, 255));

    // Nectar: YCrCb instead of HSV. Red sits at BOTH ends of the HSV hue circle (0 and 180), and a
    // single HSV range can't wrap around, so red is easier in YCrCb.
    // Y = brightness, Cr = how red, Cb = how blue.
    private static final ColorRange NECTAR_RED_RANGE = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar(32, 170, 0),     // raise the min Cr (middle number) to be pickier about "red"
            new Scalar(255, 255, 120));
    private static final ColorRange NECTAR_BLUE_RANGE = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar(16, 0, 160),     // raise the min Cb (last number) to be pickier about "blue"
            new Scalar(255, 120, 255));

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // FILTERS. These decide what counts as a real ball
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    private static final double MIN_RADIUS_PX = 4.0;    // smaller than this is too few pixels to trust
    private static final double MAX_RADIUS_PX = 150.0;  // bigger than this is usually a merged blob or a wall
    private static final double MIN_AREA_PX = 50.0;     // tiny specks of noise
    private static final double MAX_AREA_PX = 80000.0;  // huge regions that can't be one ball
    private static final double MIN_CIRCULARITY = 0.6;  // 1.0 = perfect circle. Raised from 0.4, which let almost anything through

    // How much the size method and the floor method are allowed to disagree.
    // ratio = sizeRange / floorRange. 1.0 = perfect agreement.
    // Below the min: the blob looks too BIG for where it is (usually two balls merged, or a big non-ball object).
    // Above the max: the blob looks too SMALL for where it is (usually only part of the ball got detected).
    // If real balls are getting rejected, loosen these a bit; if junk gets through, tighten them.
    private static final double MIN_SIZE_FLOOR_RATIO = 0.6;
    private static final double MAX_SIZE_FLOOR_RATIO = 1.6;

    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // MORPHOLOGY (cleaning up the blob mask)
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // Was 15. Kernels that big glue nearby blobs together (two balls become one big "ball", which
    // makes the distance way too short) and bend the shape so circularity stops meaning anything.
    private static final int MORPH_SIZE = 5;

    private VisionPortal portal;

    /** Everything we know about one accepted ball. */
    private static class Detection {
        String label;       // "Pollen" or "Nectar"
        Circle circle;      // circle fit in the image (pixels)
        double floorRange;  // straight-line distance lens -> ball center from the floor method (in)
        double sizeRange;   // same distance from the size method (in)
        double bearingDeg;  // left/right angle from the camera's center line (+ = left)
        double robotFwd;    // ball position in the ROBOT frame: inches forward of robot center
        double robotLeft;   // ball position in the ROBOT frame: inches left of robot center
    }

    // ---------------------------------------------------------------------------------------------
    // CAMERA SETUP
    // ---------------------------------------------------------------------------------------------

    /** Waits for the camera to start streaming, then locks exposure, gain, white balance and focus. */
    private void lockCameraSettings() {
        // Camera controls only work once the camera is streaming, so wait for that first.
        while (!isStopRequested() && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "waiting for stream");
            telemetry.update();
            sleep(20);
        }

        // Manual exposure: stops the camera from brightening/darkening on its own, which shifts
        // colors in and out of our ranges and causes flickering detections.
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        if (exposure != null) {
            exposure.setMode(ExposureControl.Mode.Manual);
            sleep(50); // give the camera a moment to switch modes before sending the value
            exposure.setExposure(EXPOS_MS, TimeUnit.MILLISECONDS);
        }

        // Gain: brightens the image electronically to make up for the short exposure.
        GainControl gain = portal.getCameraControl(GainControl.class);
        if (gain != null) {
            gain.setGain(GAIN);
        }

        // Manual white balance: keeps colors the same as the robot moves under different lights.
        WhiteBalanceControl whiteBalance = portal.getCameraControl(WhiteBalanceControl.class);
        if (whiteBalance != null) {
            whiteBalance.setMode(WhiteBalanceControl.Mode.MANUAL);
            sleep(50);
            whiteBalance.setWhiteBalanceTemperature(WHITE_BALANCE);
        }

        // Fixed focus: stops autofocus from hunting, which blurs the image and changes blob sizes.
        FocusControl focus = portal.getCameraControl(FocusControl.class);
        if (focus != null) {
            focus.setMode(FocusControl.Mode.Fixed);
            sleep(50);
            focus.setFocusLength(FOCUS_VAL);
        }
    }

    /**
     * Top edge of the search region. A ball on the floor can never appear above the horizon line
     * in the image, so anything up there (walls, lights, people) is ignored before it's even processed.
     * The horizon sits FOCAL_PX * tan(pitch) pixels above the image center when the camera tilts down.
     */
    private static double roiTop() {
        double horizonPxAboveCenter = FOCAL_PX * Math.tan(Math.toRadians(CAM_PITCH_DEG));
        double horizonUnity = horizonPxAboveCenter / (FRAME_HEIGHT / 2.0); // convert to -1..1 image units
        // Keep the old 0.75 edge crop as the most we'll ever look at, and never let the top go
        // below the bottom edge of the region.
        return Math.max(-0.5, Math.min(0.75, horizonUnity));
    }

    /** Builds one color locator. Pollen and Nectar use the same settings except color and outline color. */
    private static ColorBlobLocatorProcessor buildLocator(ColorRange range, int circleColor) {
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(range)
                // Only the outer outline of each blob; holes inside it don't matter.
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                // Search region in -1..1 image units (left, top, right, bottom). Sides and bottom
                // crop the noisy image edges; the top is cut at the horizon.
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, roiTop(), 0.75, -0.75))
                .setDrawContours(true)            // draw the raw outline on the camera stream for tuning
                .setBoxFitColor(0)                // 0 = don't draw the rectangle fit
                .setCircleFitColor(circleColor)   // draw the circle fit (this circle's radius feeds the size method)
                .setBlurSize(5)                   // smooth pixel noise before thresholding
                // CLOSING = dilate then erode: fills small holes (glare spots) without growing the blob overall.
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .setDilateSize(MORPH_SIZE)
                .setErodeSize(MORPH_SIZE)
                .build();
    }

    // ---------------------------------------------------------------------------------------------
    // GEOMETRY
    // ---------------------------------------------------------------------------------------------

    /**
     * Works out where a blob is, using the floor method, checks it against the size method,
     * and converts it into the robot frame. Returns null if the blob can't be a real ball.
     */
    private static Detection locate(Circle c, double diamIn, String label) {
        // Pixel offsets from the image center. +u = right in the image, +v = down in the image.
        double u = c.getX() - FRAME_WIDTH / 2.0;
        double v = c.getY() - FRAME_HEIGHT / 2.0;

        // Build the 3D direction of the ray from the lens through this pixel, in a LEVEL frame
        // (forward / down / right), by undoing the camera's downward tilt.
        //   A pixel at the image center points down at exactly the pitch angle.
        //   Pixels lower in the image (+v) point further down.
        double pitch = Math.toRadians(CAM_PITCH_DEG);
        double rayFwd   = FOCAL_PX * Math.cos(pitch) - v * Math.sin(pitch);
        double rayDown  = FOCAL_PX * Math.sin(pitch) + v * Math.cos(pitch);
        double rayRight = u;

        // If the ray doesn't point down toward the floor, it can't be hitting a ball on the floor.
        if (rayDown <= 0 || rayFwd <= 0) return null;

        // The ball's CENTER is one radius above the floor, so the ray has to drop this far to reach it.
        double drop = CAM_HEIGHT_IN - diamIn / 2.0;
        if (drop <= 0) return null; // camera is lower than the ball center; floor method can't work

        // Stretch the ray until it has dropped exactly that far. t is the scale factor.
        double t = drop / rayDown;
        double camFwd  = t * rayFwd;     // inches forward of the lens (level, along camera heading)
        double camLeft = -t * rayRight;  // inches left of the lens (minus because +u was right)

        // Straight-line distance from lens to ball center according to the floor method.
        double floorRange = t * Math.sqrt(rayFwd * rayFwd + rayDown * rayDown + rayRight * rayRight);

        // Straight-line distance according to the size method (similar triangles on the known diameter).
        double sizeRange = (diamIn * FOCAL_PX) / (2.0 * c.getRadius());

        // Cross-check: if the blob's apparent size doesn't match how far away the floor says it is,
        // it's a merged blob, a partial detection, or not a ball. Reject it.
        double ratio = sizeRange / floorRange;
        if (ratio < MIN_SIZE_FLOOR_RATIO || ratio > MAX_SIZE_FLOOR_RATIO) return null;

        // Rotate by the camera's yaw on the robot, then add the camera's mount offset.
        // Same camera -> robot conversion as before, just fed with the better numbers.
        double yaw = Math.toRadians(CAM_YAW_DEG);
        Detection d = new Detection();
        d.label = label;
        d.circle = c;
        d.floorRange = floorRange;
        d.sizeRange = sizeRange;
        d.bearingDeg = Math.toDegrees(Math.atan2(camLeft, camFwd));
        d.robotFwd  = CAM_OFFSET_X + camFwd * Math.cos(yaw) - camLeft * Math.sin(yaw);
        d.robotLeft = CAM_OFFSET_Y + camFwd * Math.sin(yaw) + camLeft * Math.cos(yaw);
        return d;
    }

    /**
     * Runs all the filters on one processor's blobs and returns the best real ball, or null.
     * Blobs come back sorted largest first, so the first one that survives is usually the closest.
     */
    private static Detection findBest(ColorBlobLocatorProcessor locator, double diamIn, String label) {
        // Grab the list ONCE per loop and copy it. Calling getBlobs() several times can hand back
        // lists from different frames, which is how the old code sometimes mixed up blobs.
        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>(locator.getBlobs());

        // Throw out blobs that are too small, too big, or not round enough.
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, MIN_AREA_PX, MAX_AREA_PX, blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, MIN_CIRCULARITY, 1.0, blobs);

        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            Circle c = blob.getCircle();
            // Radius limits (these were declared in the old code but never actually used).
            if (c.getRadius() < MIN_RADIUS_PX || c.getRadius() > MAX_RADIUS_PX) continue;

            // Geometry checks: must be below the horizon and size must match distance.
            Detection d = locate(c, diamIn, label);
            if (d != null) return d;
        }
        return null; // nothing passed every check
    }

    // ---------------------------------------------------------------------------------------------
    // MAIN
    // ---------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        // Pollen outlined in magenta, Nectar in cyan, so you can tell them apart on the stream.
        ColorBlobLocatorProcessor pollenLocator = buildLocator(POLLEN_RANGE, Color.rgb(255, 0, 255));
        ColorBlobLocatorProcessor nectarLocator = buildLocator(
                (Alliance.current == Alliance.BLUE) ? NECTAR_BLUE_RANGE : NECTAR_RED_RANGE,
                Color.rgb(0, 255, 255));

        // One camera, both processors running on every frame.
        WebcamName webcam = hardwareMap.get(WebcamName.class, CAMERA_NAME);
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(FRAME_WIDTH, FRAME_HEIGHT))
                .addProcessors(pollenLocator, nectarLocator)
                .build();

        lockCameraSettings();

        // Runs during init too, so you can tune on the Camera Stream before pressing start.
        while (opModeIsActive() || opModeInInit()) {
            // Nectar first (it's the priority target), Pollen if there's no good Nectar.
            Detection target = findBest(nectarLocator, NECTAR_DIAM_IN, "Nectar");
            if (target == null) {
                target = findBest(pollenLocator, POLLEN_DIAM_IN, "Pollen");
            }

            telemetry.addData("Camera", portal.getCameraState());
            telemetry.addData("Focal px", "%.1f %s", FOCAL_PX,
                    FOCAL_PX_CALIBRATED > 0 ? "(calibrated)" : "(FOV guess)");
            // Raw counts BEFORE filtering, useful for seeing how much junk the filters remove.
            telemetry.addData("Raw blobs", "nectar=%d  pollen=%d",
                    nectarLocator.getBlobs().size(), pollenLocator.getBlobs().size());

            if (target != null) {
                Circle c = target.circle;
                double diam = target.label.equals("Nectar") ? NECTAR_DIAM_IN : POLLEN_DIAM_IN;

                telemetry.addData("Target", "%s  x=%.0f y=%.0f r=%.1f px",
                        target.label, c.getX(), c.getY(), c.getRadius());
                // Both distances side by side: if they drift apart, re-check pitch, height, or focal.
                telemetry.addData("Range", "floor=%.1f in  size=%.1f in",
                        target.floorRange, target.sizeRange);
                telemetry.addData("Bearing", "%.1f deg", target.bearingDeg);
                telemetry.addData("Robot frame", "fwd=%.1f  left=%.1f in",
                        target.robotFwd, target.robotLeft);
                // Calibration helper: with the ball at CALIB_DIST_IN from the lens, this is the focal
                // length that makes the size method exactly right.
                telemetry.addData("Focal if @" + CALIB_DIST_IN + " in", "%.1f",
                        CALIB_DIST_IN * 2.0 * c.getRadius() / diam);
            } else {
                telemetry.addData("Target", "none passed the filters");
            }

            telemetry.update();
            sleep(50);
        }
        portal.close();
    }
}
