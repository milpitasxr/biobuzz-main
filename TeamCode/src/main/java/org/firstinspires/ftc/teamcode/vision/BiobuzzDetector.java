package org.firstinspires.ftc.teamcode.vision;

import android.content.res.AssetManager;
import java.io.*;
import java.nio.*;
import java.util.*;
import java.util.concurrent.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.Interpreter;

/** MobileNet crop classifier plus the desktop detector's color/shape proposals. */
public final class BiobuzzDetector implements AutoCloseable {
  public static final String[] LABELS = {
    "Background", "Pollen yellow", "Nectar red", "Nectar blue"
  };
  public static final double[] DIAMETERS = {0, .07112, .09144, .09144};
  // Single-worker mode runs inline; an optional pool supports the older parallel mode.
  private final ExecutorService pool;
  private final Classifier[] classifiers;
  // Summed-area tables answer rectangle color queries with four array reads.
  private final int[][] integralColors = new int[3][];
  // Reuse Java arrays instead of allocating several camera-sized buffers each frame.
  private final Pixels hsvPixels = new Pixels();
  private final Pixels[] maskPixels = {new Pixels(), new Pixels(), new Pixels()};
  private final Pixels[] corePixels = {new Pixels(), new Pixels(), new Pixels()};
  // A permissive body mask prevents saturated holes from becoming separate objects.
  private final Pixels[] bodyPixels = {new Pixels(), new Pixels(), new Pixels()};
  // Reusable grayscale and edge buffers support the circular-outline check.
  private final Mat gray = new Mat(), gradientX = new Mat(), gradientY = new Mat();
  private short[] gx = new short[0], gy = new short[0];
  private int imageWidth, imageHeight;
  private int integralStride;
  // These CPU totals add parallel jobs together; they are not elapsed frame time.
  public double lastModelMs = 0, lastCropMs = 0;
  public int lastInvocations = 0;
  // Opt-in diagnostics for saved-image tests; disabled during normal live processing.
  public boolean traceEnabled = false;
  public final List<String> trace = new ArrayList<>();
  private final Mat openKernel = Mat.ones(3, 3, CvType.CV_8U),
      closeKernel = Mat.ones(5, 5, CvType.CV_8U);
  public int lastCandidateCount = 0;
  public double lastProposalMs = 0, lastInferenceMs = 0;
  // Per-color profiling order is yellow, red, blue; all values are milliseconds.
  public final double[] colorMs = new double[3], splitMs = new double[3], outlineMs = new double[3];
  public double lastPostMs = 0;
  // 640-pixel webcam images stay at full resolution with this default.
  public int proposalWidth = 800;
  // Precompute angles used around candidate boundaries, avoiding repeated trig calls.
  private static final double[] COS96 = new double[96],
      SIN96 = new double[96],
      COS64 = new double[64],
      SIN64 = new double[64];
  private static final double[] BOUNDARY_FACTORS = {1.12, 1.25};

  static {
    for (int i = 0; i < 96; i++) {
      double angle = i * 2 * Math.PI / 96;
      COS96[i] = Math.cos(angle);
      SIN96[i] = Math.sin(angle);
    }
    // Sample around an ellipse matching the candidate box; tolerate small boundary errors.
    for (int i = 0; i < 64; i++) {
      double angle = i * 2 * Math.PI / 64;
      COS64[i] = Math.cos(angle);
      SIN64[i] = Math.sin(angle);
    }
  }

  // One accepted detection. The coordinates are relative to the current camera.
  public static final class Ball {
    public final Rect box;
    public final int cls;
    public final double score, quality, x, y, z, range, azimuth, elevation;

    Ball(Rect box, int cls, double score, double quality, int width, int height, double fov) {
      this.box = box;
      this.cls = cls;
      this.score = score;
      this.quality = quality;
      // Convert horizontal field of view into focal length measured in image pixels.
      double f = width / (2 * Math.tan(Math.toRadians(fov) / 2));
      // Pinhole model: depth = focal length × known ball diameter / pixel diameter.
      z = f * DIAMETERS[cls] / Math.max(box.width, box.height);
      // Project the box center onto camera axes: right is +X, up is +Y, forward is +Z.
      x = (box.x + box.width / 2. - width / 2.) * z / f;
      y = (height / 2. - box.y - box.height / 2.) * z / f;
      // Range is the straight-line distance; Z above is only the forward component.
      range = Math.sqrt(x * x + y * y + z * z);
      // Horizontal angle is positive right; vertical angle is positive up.
      azimuth = Math.toDegrees(Math.atan2(x, z));
      elevation = Math.toDegrees(Math.atan2(y, Math.hypot(x, z)));
    }
  }

  // A possible ball outline, not yet accepted by the CNN.
  private static final class Candidate {
    final Rect box;
    final int cls;
    final double area;

    Candidate(Rect b, int c, double a) {
      box = b;
      cls = c;
      area = a;
    }
  }

  public BiobuzzDetector(AssetManager assets, boolean int8) throws IOException {
    this(assets, int8, 1);
  }

  /** Independent single-thread interpreters; no interpreter is used concurrently. */
  public BiobuzzDetector(AssetManager assets, boolean int8, int workers) throws IOException {
    this(assets, int8, workers, 96);
  }

  /** Crop resolution is independent of the camera resolution and proposal search size. */
  public BiobuzzDetector(AssetManager assets, boolean int8, int workers, int inputSize)
      throws IOException {
    // Only permit precision/size combinations whose model assets ship with the application.
    boolean supportedSize = inputSize == 96 || inputSize == 128 || (!int8 && inputSize == 160);
    if (!supportedSize)
      throw new IllegalArgumentException("Choose FP32 96/128/160, or INT8 96/128");
    int count = Math.max(1, Math.min(3, workers));
    classifiers = new Classifier[count];
    try {
      for (int i = 0; i < count; i++) classifiers[i] = new Classifier(assets, int8, inputSize);
    } catch (IOException | RuntimeException e) {
      for (Classifier c : classifiers) if (c != null) c.close();
      throw e;
    }
    pool =
        count == 1
            ? null
            : Executors.newFixedThreadPool(
                count,
                r -> {
                  Thread t = new Thread(r, "BIOBUZZ-compute");
                  t.setDaemon(true);
                  return t;
                });
  }

  // Submit a batch and wait for every job before its shared native images can be freed.
  private <T> List<T> runAll(List<Callable<T>> jobs) {
    // Serial mode stays on the owning detector thread: no executor, futures, or parallel jobs.
    if (pool == null) {
      List<T> values = new ArrayList<>(jobs.size());
      for (Callable<T> job : jobs) {
        try {
          values.add(job.call());
        } catch (Exception e) {
          if (e instanceof InterruptedException) Thread.currentThread().interrupt();
          throw new IllegalStateException(e);
        }
      }
      return values;
    }
    List<Future<T>> futures = new ArrayList<>();
    for (Callable<T> job : jobs) futures.add(pool.submit(job));
    List<T> values = new ArrayList<>();
    boolean interrupted = false;
    Throwable failure = null;
    // Even on failure/interruption, let native calls finish before releasing
    // their shared images or closing their interpreters.
    for (Future<T> f : futures) {
      boolean finished = false;
      while (!finished) {
        try {
          values.add(f.get());
          finished = true;
        } catch (InterruptedException e) {
          interrupted = true;
        } catch (ExecutionException e) {
          failure = e.getCause();
          finished = true;
        }
      }
    }
    if (interrupted) Thread.currentThread().interrupt();
    if (failure != null) throw new IllegalStateException(failure);
    return values;
  }

  // OpenCV HSV uses hue 0–179 and saturation/value 0–255. Red wraps around hue zero.
  private static boolean color(int h, int s, int v, int cls) {
    if (cls == 1) return h >= 17 && h <= 42 && s >= 45 && v >= 45;
    if (cls == 2) return (h <= 12 || h >= 167) && s >= 100 && v >= 40;
    return h >= 90 && h <= 135 && s >= 105 && v >= 35;
  }

  // Build a binary image: white pixels match the requested ball color.
  private static Mat mask(Mat hsv, int cls) {
    Mat m = new Mat();
    if (cls == 1) Core.inRange(hsv, new Scalar(17, 45, 45), new Scalar(42, 255, 255), m);
    else if (cls == 3) Core.inRange(hsv, new Scalar(90, 105, 35), new Scalar(135, 255, 255), m);
    else {
      Mat other = new Mat();
      Core.inRange(hsv, new Scalar(0, 100, 40), new Scalar(12, 255, 255), m);
      Core.inRange(hsv, new Scalar(167, 100, 40), new Scalar(179, 255, 255), other);
      Core.bitwise_or(m, other, m);
      other.release();
    }
    return m;
  }

  // Find outer outlines only; holes inside a ball do not become separate objects.
  private static List<MatOfPoint> contours(Mat m) {
    List<MatOfPoint> out = new ArrayList<>();
    Mat hierarchy = new Mat();
    // OpenCV 4 findContours does not modify its source, so no full-image clone is needed.
    Imgproc.findContours(m, out, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
    hierarchy.release();
    return out;
  }

  // Reject filled, straight-sided patches before generating circle hypotheses from them.
  // A drawn box around a real ball is mostly empty at its corners, so it is not a filled box.
  private static boolean boxLike(MatOfPoint contour, Pixels support) {
    Rect r = Imgproc.boundingRect(contour);
    if (Math.min(r.width, r.height) < 12) return false;
    MatOfPoint2f curve = new MatOfPoint2f(contour.toArray()), polygon = new MatOfPoint2f();
    boolean straight;
    try {
      Imgproc.approxPolyDP(curve, polygon, .025 * Imgproc.arcLength(curve, true), true);
      straight =
          polygon.total() <= 6
              && Imgproc.contourArea(polygon) >= .94 * Imgproc.contourArea(contour);
    } finally {
      curve.release();
      polygon.release();
    }
    if (!straight) return false;
    int foreground = 0, samples = 0;
    int dx = Math.max(2, r.width / 5), dy = Math.max(2, r.height / 5);
    // Ellipse corners should largely be background; rectangular faces occupy those corners.
    for (int cy : new int[] {r.y, r.y + r.height - dy})
      for (int cx : new int[] {r.x, r.x + r.width - dx})
        for (int y = cy; y < cy + dy; y++)
          for (int x = cx; x < cx + dx; x++) {
            samples++;
            if (support.data[y * support.width + x] != 0) foreground++;
          }
    return foreground > .60 * samples;
  }

  // Bulk-copy native pixels once, then sample them cheaply from Java.
  private static final class Pixels {
    byte[] data = new byte[0];
    int width, height, channels;

    Pixels read(Mat m) {
      width = m.cols();
      height = m.rows();
      channels = m.channels();
      int size = width * height * channels;
      if (data.length != size) data = new byte[size];
      m.get(0, 0, data);
      return this;
    }

    boolean isColor(int x, int y, int cls) {
      int i = (y * width + x) * 3;
      return color(data[i] & 255, data[i + 1] & 255, data[i + 2] & 255, cls);
    }
  }

  // A real outline should have mostly non-matching color just outside its perimeter.
  private static boolean boundary(Pixels mask, double a, double b, double c, double d) {
    int count = 0, inside = 0;
    double cx = (a + c) / 2, cy = (b + d) / 2;
    for (double factor : BOUNDARY_FACTORS)
      for (int i = 0; i < 96; i++) {
        int x = (int) Math.rint(cx + (c - a) / 2 * factor * COS96[i]),
            y = (int) Math.rint(cy + (d - b) / 2 * factor * SIN96[i]);
        if (x >= 0 && x < mask.width && y >= 0 && y < mask.height) {
          count++;
          if (mask.data[y * mask.width + x] != 0) inside++;
        }
      }
    return count >= 80 && inside < .45 * count;
  }

  // Convert proposal coordinates back to full-size pixels and clip to the image.
  private static Rect bounds(double a, double b, double c, double d, double scale, int w, int h) {
    int x = Math.max(0, (int) (a / scale)), y = Math.max(0, (int) (b / scale));
    return new Rect(
        x,
        y,
        Math.max(0, Math.min(w, (int) Math.ceil(c / scale)) - x),
        Math.max(0, Math.min(h, (int) Math.ceil(d / scale)) - y));
  }

  // Area shared by two rectangles, or zero when they do not overlap.
  private static double intersection(Rect a, Rect b) {
    return Math.max(0, Math.min(a.x + a.width, b.x + b.width) - Math.max(a.x, b.x))
        * Math.max(0, Math.min(a.y + a.height, b.y + b.height) - Math.max(a.y, b.y));
  }

  // Intersection-over-union measures how similar two proposed boxes are.
  private static double iou(Rect a, Rect b) {
    double inter = intersection(a, b);
    return inter / Math.max(1, a.area() + b.area() - inter);
  }

  // Reject weak boundaries and tiny boxes before placing a proposal in the candidate list.
  private static void add(
      List<Candidate> out,
      Pixels mask,
      double a,
      double b,
      double c,
      double d,
      double scale,
      int w,
      int h,
      int cls,
      double area,
      boolean dedup) {
    if (!boundary(mask, a, b, c, d)) return;
    Rect box = bounds(a, b, c, d, scale, w, h);
    if (Math.min(box.width, box.height) < 12) return;
    if (dedup) for (Candidate p : out) if (p.cls == cls && iou(box, p.box) > .92) return;
    out.add(new Candidate(box, cls, area));
  }

  // Stage 1: search the three colors concurrently, then combine their candidate outlines.
  private List<Candidate> proposals(Mat rgb, Mat fullHsv) {
    int w = rgb.cols(), h = rgb.rows();
    double scale = Math.min(1., Math.max(320, Math.min(800, proposalWidth)) / (double) w);
    Mat small = new Mat();
    final Mat hsv = scale == 1 ? fullHsv : new Mat();
    if (scale != 1) {
      Imgproc.resize(rgb, small, new Size(Math.rint(w * scale), Math.rint(h * scale)));
      Imgproc.cvtColor(small, hsv, Imgproc.COLOR_RGB2HSV);
    }
    List<Callable<List<Candidate>>> jobs = new ArrayList<>();
    for (int cls = 1; cls <= 3; cls++) {
      final int colorClass = cls;
      jobs.add(() -> proposalsForColor(hsv, scale, w, h, colorClass));
    }
    List<Candidate> out = new ArrayList<>();
    try {
      for (List<Candidate> colorCandidates : runAll(jobs)) out.addAll(colorCandidates);
    } finally {
      if (hsv != fullHsv) hsv.release();
      small.release();
    }
    // Cap pathological scenes at 64 proposals, preferring larger connected regions.
    out.sort((a, b) -> Double.compare(b.area, a.area));
    return out.subList(0, Math.min(64, out.size()));
  }

  // Generate several plausible outlines so touching and partly obscured balls can survive.
  private List<Candidate> proposalsForColor(Mat hsv, double scale, int w, int h, int cls) {
    long begin = System.nanoTime();
    List<Candidate> out = new ArrayList<>();
    Mat m = cls == 1 ? new Mat() : mask(hsv, cls);
    // Yellow's permissive mask merges with pale painted walls. Locate
    // saturated body arcs first; retain compact pale fallback below.
    if (cls == 1) Core.inRange(hsv, new Scalar(17, 90, 45), new Scalar(42, 255, 255), m);
    // Opening removes isolated noise; closing reconnects small gaps in the colored surface.
    Imgproc.morphologyEx(m, m, Imgproc.MORPH_OPEN, openKernel);
    Imgproc.morphologyEx(m, m, Imgproc.MORPH_CLOSE, closeKernel);
    Pixels mData = maskPixels[cls - 1].read(m);
    Mat body = cls == 1 ? mask(hsv, cls) : null;
    Pixels bodyData = mData;
    if (body != null) {
      Imgproc.morphologyEx(body, body, Imgproc.MORPH_CLOSE, closeKernel);
      bodyData = bodyPixels[cls - 1].read(body);
    }
    List<MatOfPoint> all = contours(m);
    for (MatOfPoint contour : all) {
      Rect r = Imgproc.boundingRect(contour);
      double area = Imgproc.contourArea(contour),
          aspect = (double) r.width / r.height,
          extent = area / r.area();
      if (Math.min(r.width, r.height) < 12 * scale || area < 60 || extent < .28) continue;
      if (boxLike(contour, mData)) continue;
      // Fill the outer contour so holes in the plastic do not split the ball into pieces.
      Mat filled = Mat.zeros(r.height + 2, r.width + 2, CvType.CV_8U),
          dist = new Mat(),
          dilated = new Mat();
      Point[] shifted = contour.toArray();
      for (Point p : shifted) {
        p.x -= r.x - 1;
        p.y -= r.y - 1;
      }
      MatOfPoint local = new MatOfPoint(shifted);
      Imgproc.drawContours(filled, Collections.singletonList(local), -1, new Scalar(255), -1);
      // Bound expensive peak splitting for large background blobs.
      // Color masks and final contour boxes retain full resolution.
      double gridScale = Math.min(1., 96. / Math.max(filled.cols(), filled.rows()));
      if (gridScale < 1)
        Imgproc.resize(
            filled,
            filled,
            new Size(
                Math.max(2, Math.rint(filled.cols() * gridScale)),
                Math.max(2, Math.rint(filled.rows() * gridScale))),
            0,
            0,
            Imgproc.INTER_NEAREST);
      double sx = (r.width + 2.) / filled.cols(),
          sy = (r.height + 2.) / filled.rows(),
          sr = Math.sqrt(sx * sy);
      // Distance to the outline gives possible ball radii; multiple peaks can mean touching balls.
      Imgproc.distanceTransform(filled, dist, Imgproc.DIST_L2, 5);
      Core.MinMaxLocResult mm = Core.minMaxLoc(dist);
      double radius = mm.maxVal;
      // Suppress nearby distance peaks, avoiding many proposals for the same ball.
      int peakSize = Math.max(5, ((int) (radius * .9)) | 1);
      Mat kernel = Mat.ones(peakSize, peakSize, CvType.CV_8U);
      Imgproc.dilate(dist, dilated, kernel);
      kernel.release();
      Mat maxima = new Mat(), high = new Mat();
      Core.subtract(dilated, new Scalar(.01), dilated);
      Core.compare(dist, dilated, maxima, Core.CMP_GE);
      Core.compare(dist, new Scalar(radius * .48), high, Core.CMP_GT);
      Core.bitwise_and(maxima, high, maxima);
      high.release();
      Mat labels = new Mat(), stats = new Mat(), centers = new Mat();
      // Group plateau pixels into single peaks, then map their centers back to full resolution.
      int count = Imgproc.connectedComponentsWithStats(maxima, labels, stats, centers);
      List<double[]> peaks = new ArrayList<>(), selected = new ArrayList<>();
      for (int i = 1; i < count; i++) {
        double cx = centers.get(i, 0)[0], cy = centers.get(i, 1)[0];
        peaks.add(
            new double[] {
              dist.get(
                          Math.min(dist.rows() - 1, (int) Math.rint(cy)),
                          Math.min(dist.cols() - 1, (int) Math.rint(cx)))[0]
                  * sr,
              (cx + .5) * sx - .5,
              (cy + .5) * sy - .5
            });
      }
      peaks.sort((a, b) -> Double.compare(b[0], a[0]));
      for (double[] peak : peaks) {
        boolean keep = true;
        for (double[] p : selected)
          if (Math.hypot(peak[1] - p[1], peak[2] - p[2]) <= Math.max(peak[0], p[0]) * 1.15) {
            keep = false;
            break;
          }
        if (keep) selected.add(peak);
      }
      // Separate touching blobs using their distinct interior peaks.
      if (selected.size() > 1)
        for (int i = 0; i < Math.min(12, selected.size()); i++) {
          double[] p = selected.get(i);
          double cx = r.x + p[1] - 1, cy = r.y + p[2] - 1;
          add(
              out, mData, cx - p[0], cy - p[0], cx + p[0], cy + p[0], scale, w, h, cls, area,
              false);
        }
      else if (aspect > .52 && aspect < 1.92) {
        MatOfPoint2f points = new MatOfPoint2f(contour.toArray());
        double perimeter = Imgproc.arcLength(points, true);
        points.release();
        // Circularity rejects long, irregular color patches before the expensive CNN stage.
        if (4 * Math.PI * area / Math.max(1, perimeter * perimeter) > .3) {
          double cx = r.x + (mm.maxLoc.x + .5) * sx - 1.5,
              cy = r.y + (mm.maxLoc.y + .5) * sy - 1.5,
              rr = radius * sr * 1.04;
          if (extent > .68 && aspect > .65 && aspect < 1.5)
            add(out, mData, r.x, r.y, r.x + r.width, r.y + r.height, scale, w, h, cls, area, false);
          else
            add(
                out,
                mData,
                Math.max(r.x, cx - rr),
                Math.max(r.y, cy - rr),
                Math.min(r.x + r.width, cx + rr),
                Math.min(r.y + r.height, cy + rr),
                scale,
                w,
                h,
                cls,
                area,
                false);
        }
      }
      for (Mat mat : Arrays.asList(local, filled, dist, dilated, maxima, labels, stats, centers))
        mat.release();
    }
    // Keep these contours for the outline pass instead of extracting them twice.
    List<MatOfPoint> primaryContours = all;
    splitMs[cls - 1] = (System.nanoTime() - begin) / 1e6;
    long outlineStart = System.nanoTime();
    // Also inspect strongly saturated pixels, which can separate a ball from a pale background.
    Mat core = m.clone(), sat = new Mat();
    Core.extractChannel(hsv, sat, 1);
    Imgproc.threshold(sat, sat, 189, 255, Imgproc.THRESH_BINARY);
    Core.bitwise_and(core, sat, core);
    sat.release();
    // Keep both tight rectangular outlines and enclosing circles for partly visible balls.
    for (Mat support : Arrays.asList(m, core)) {
      Pixels supportData = support == m ? mData : corePixels[cls - 1].read(support);
      all = support == m ? primaryContours : contours(support);
      for (MatOfPoint contour : all) {
        Rect r = Imgproc.boundingRect(contour);
        double area = Imgproc.contourArea(contour), ratio = (double) r.width / r.height;
        if (Math.min(r.width, r.height) < Math.max(16, 12 * scale)
            || ratio <= .55
            || ratio >= 1.8
            || area / r.area() < .38
            || Math.max(ratio, 1 / ratio) > 1.5) continue;
        if (boxLike(contour, supportData)) continue;
        Point center = new Point();
        float[] radius = new float[1];
        MatOfPoint2f points = new MatOfPoint2f(contour.toArray());
        Imgproc.minEnclosingCircle(points, center, radius);
        points.release();
        add(
            out,
            support == core && cls == 1 ? bodyData : supportData,
            r.x,
            r.y,
            r.x + r.width,
            r.y + r.height,
            scale,
            w,
            h,
            cls,
            area,
            true);
        add(
            out,
            support == core && cls == 1 ? bodyData : supportData,
            center.x - radius[0],
            center.y - radius[0],
            center.x + radius[0],
            center.y + radius[0],
            scale,
            w,
            h,
            cls,
            area,
            true);
      }
      for (Mat mat : all) mat.release();
    }
    core.release();
    if (body != null) body.release();
    m.release();
    if (cls == 1) {
      // Fallback for washed-out yellow: only compact regions without an existing saturated body.
      Mat pale = mask(hsv, cls);
      Imgproc.morphologyEx(pale, pale, Imgproc.MORPH_OPEN, openKernel);
      Imgproc.morphologyEx(pale, pale, Imgproc.MORPH_CLOSE, closeKernel);
      Pixels paleData = corePixels[0].read(pale);
      all = contours(pale);
      for (MatOfPoint contour : all) {
        Rect r = Imgproc.boundingRect(contour);
        double area = Imgproc.contourArea(contour), ratio = r.width / (double) r.height;
        if (Math.min(r.width, r.height) < 12 * scale
            || area < 60
            || ratio < .6
            || ratio > 1.6
            || area / r.area() < .45) continue;
        Rect full = bounds(r.x, r.y, r.x + r.width, r.y + r.height, scale, w, h);
        boolean hasCore = false;
        for (Candidate c : out)
          if (c.box.area() > .45 * full.area()
              && intersection(full, c.box) / c.box.area() > .85
              && edgeSupport(c.box, true) >= .40) {
            hasCore = true;
            break;
          }
        if (hasCore) continue; // Never expand a body outline to include its attached wall.
        if (boxLike(contour, paleData)) continue;
        add(out, paleData, r.x, r.y, r.x + r.width, r.y + r.height, scale, w, h, cls, area, true);
        // A bright body can have labels/shadows attached to its contour. Its largest filled
        // interior circle is another whole-body hypothesis, independent of saturated holes.
        Mat filled = Mat.zeros(r.height + 2, r.width + 2, CvType.CV_8U), distance = new Mat();
        Point[] shifted = contour.toArray();
        for (Point point : shifted) {
          point.x -= r.x - 1;
          point.y -= r.y - 1;
        }
        MatOfPoint local = new MatOfPoint(shifted);
        Imgproc.drawContours(filled, Collections.singletonList(local), -1, new Scalar(255), -1);
        Imgproc.distanceTransform(filled, distance, Imgproc.DIST_L2, 5);
        Core.MinMaxLocResult peak = Core.minMaxLoc(distance);
        double bodyX = r.x + peak.maxLoc.x - 1, bodyY = r.y + peak.maxLoc.y - 1;
        double bodyRadius = peak.maxVal * 1.04;
        add(
            out,
            paleData,
            bodyX - bodyRadius,
            bodyY - bodyRadius,
            bodyX + bodyRadius,
            bodyY + bodyRadius,
            scale,
            w,
            h,
            cls,
            area,
            true);
        local.release();
        filled.release();
        distance.release();
        Point center = new Point();
        float[] radius = new float[1];
        MatOfPoint2f points = new MatOfPoint2f(contour.toArray());
        Imgproc.minEnclosingCircle(points, center, radius);
        points.release();
        add(
            out,
            paleData,
            center.x - radius[0],
            center.y - radius[0],
            center.x + radius[0],
            center.y + radius[0],
            scale,
            w,
            h,
            cls,
            area,
            true);
      }
      for (Mat mat : all) mat.release();
      pale.release();
    }
    outlineMs[cls - 1] = (System.nanoTime() - outlineStart) / 1e6;
    colorMs[cls - 1] = (System.nanoTime() - begin) / 1e6;
    return out;
  }

  // Stage 2: sum saturation-weighted color evidence over all rectangles in constant time.
  private void buildColorSums(Mat hsv) {
    integralStride = hsv.cols() + 1;
    int count = (hsv.rows() + 1) * integralStride;
    // 255 is the maximum per-pixel contribution. Reject sizes that could overflow.
    if (hsv.total() > Integer.MAX_VALUE / 255)
      throw new IllegalArgumentException("Frame too large for color sums");
    Mat saturation = new Mat();
    Core.extractChannel(hsv, saturation, 1);
    List<Callable<Void>> jobs = new ArrayList<>();
    for (int cls = 1; cls <= 3; cls++) {
      final int colorClass = cls;
      // Each job writes a different array and owns its temporary native images.
      jobs.add(
          () -> {
            Mat m = mask(hsv, colorClass), weighted = new Mat(), sum = new Mat();
            try {
              Core.bitwise_and(saturation, m, weighted);
              Imgproc.integral(weighted, sum, CvType.CV_32S);
              if (integralColors[colorClass - 1] == null
                  || integralColors[colorClass - 1].length != count)
                integralColors[colorClass - 1] = new int[count];
              sum.get(0, 0, integralColors[colorClass - 1]);
              return null;
            } finally {
              m.release();
              weighted.release();
              sum.release();
            }
          });
    }
    // Do not free the shared saturation image until all jobs finish reading it.
    try {
      runAll(jobs);
    } finally {
      saturation.release();
    }
  }

  // Subtract the four summed-area corners to recover the requested rectangle total.
  private double colorSum(int cls, int x, int y, int width, int height) {
    int[] p = integralColors[cls - 1];
    int a = y * integralStride + x, b = (y + height) * integralStride + x;
    return (p[b + width] - p[b] - p[a + width] + p[a]) / 255.;
  }

  // Vote in the middle 60% of the box so nearby balls/background have less influence.
  private int centralColor(Rect box) {
    int dx = box.width / 5, dy = box.height / 5, w = box.width - 2 * dx, h = box.height - 2 * dy;
    double bestScore = -1;
    int best = 0;
    for (int cls = 1; cls <= 3; cls++) {
      double score = colorSum(cls, box.x + dx, box.y + dy, w, h);
      if (score > bestScore) {
        bestScore = score;
        best = cls;
      }
    }
    return bestScore > .06 * w * h ? best : 0;
  }

  // Rank outlines by color coverage inside and lack of matching color just outside.
  // A few colored pixels must not turn an otherwise unrelated crop into a ball.
  // Sample the entire rectangle; a fully visible sphere normally covers about 78% of it.
  private static double bodyCoverage(Pixels hsv, Rect box, int cls) {
    int colored = 0;
    for (int y = 0; y < 16; y++)
      for (int x = 0; x < 16; x++) {
        int px = Math.min(hsv.width - 1, box.x + (int) ((x + .5) * box.width / 16));
        int py = Math.min(hsv.height - 1, box.y + (int) ((y + .5) * box.height / 16));
        if (hsv.isColor(px, py, cls)) colored++;
      }
    return colored / 256.;
  }

  // Rank outlines by color coverage inside and lack of matching color just outside.
  private double quality(Pixels hsv, Rect r, int cls) {
    double fill = colorSum(cls, r.x, r.y, r.width, r.height);
    int n = 0, k = 0;
    // Sample around an ellipse matching the candidate box; tolerate small boundary errors.
    for (int i = 0; i < 64; i++) {
      int x = (int) Math.rint(r.x + r.width / 2. + r.width * .57 * COS64[i]),
          y = (int) Math.rint(r.y + r.height / 2. + r.height * .57 * SIN64[i]);
      if (x >= 0 && y >= 0 && x < hsv.width && y < hsv.height) {
        n++;
        if (hsv.isColor(x, y, cls)
            && (cls != 1 || (hsv.data[(y * hsv.width + x) * 3 + 1] & 255) >= 90)) k++;
      }
    }
    return Math.max(.01, Math.sqrt(fill / r.area()) * (n > 0 ? 1. - (double) k / n : 0));
  }

  // Blur small texture first, then compute horizontal/vertical brightness gradients.
  private void buildEdges(Mat rgb) {
    imageWidth = rgb.cols();
    imageHeight = rgb.rows();
    int size = imageWidth * imageHeight;
    if (gx.length != size) {
      gx = new short[size];
      gy = new short[size];
    }
    Imgproc.cvtColor(rgb, gray, Imgproc.COLOR_RGB2GRAY);
    Imgproc.GaussianBlur(gray, gray, new Size(3, 3), .8);
    Imgproc.Sobel(gray, gradientX, CvType.CV_16S, 1, 0);
    Imgproc.Sobel(gray, gradientY, CvType.CV_16S, 0, 1);
    gradientX.get(0, 0, gx);
    gradientY.get(0, 0, gy);
  }

  /** Circumference evidence, not CNN confidence: reject a ball plus a flat wall. */
  private double edgeSupport(Rect r, boolean requireDistributedOutline) {
    int visible = 0, supported = 0;
    int[] sectors = new int[8];
    double cx = r.x + r.width / 2., cy = r.y + r.height / 2.;
    // Sample around an ellipse matching the candidate box; tolerate small boundary errors.
    for (int i = 0; i < 64; i++) {
      double nx = COS64[i], ny = SIN64[i];
      boolean seen = false, edge = false;
      for (int step = -2; step <= 2; step++) {
        double radius = .5 * (1 + step * .075);
        int x = (int) Math.rint(cx + r.width * radius * nx),
            y = (int) Math.rint(cy + r.height * radius * ny);
        if (x < 1 || y < 1 || x >= imageWidth - 1 || y >= imageHeight - 1) continue;
        seen = true;
        int index = y * imageWidth + x;
        double dx = gx[index],
            dy = gy[index],
            magnitude = dx * dx + dy * dy,
            dot = dx * nx + dy * ny;
        // Require a strong edge facing roughly outward, rather than arbitrary nearby texture.
        if (magnitude >= 4096 && dot * dot >= .36 * magnitude) {
          edge = true;
          break;
        }
      }
      if (seen) visible++;
      if (edge) {
        supported++;
        sectors[i / 8]++;
      }
    }
    // A wall can supply many edges along one side. A ball needs evidence around its outline.
    // Large proposals need seven arcs: wall texture can imitate a partial circumference.
    // Small proposals allow two missing arcs because fewer edge pixels survive camera sampling.
    if (requireDistributedOutline) {
      int coveredArcs = 0;
      for (int count : sectors) if (count >= 2) coveredArcs++;
      if (coveredArcs < (Math.max(r.width, r.height) > 100 ? 7 : 6)) return 0;
    }
    return visible >= 24 ? supported / (double) visible : 0;
  }

  // Return confidence and separate crop/CNN timings from each parallel worker.
  private static final class Score {
    final double confidence, cropMs, modelMs;

    Score(double c, double p, double m) {
      confidence = c;
      cropMs = p;
      modelMs = m;
    }
  }

  // Each worker owns this interpreter and its buffers: TFLite interpreters are not shared.
  private static final class Classifier implements AutoCloseable {
    private final Interpreter model;
    private final ByteBuffer input, output;
    private final boolean quantized;
    private final Mat resizedCrop = new Mat();
    private final int size;
    private final byte[] cropPixels, quantizedLookup = new byte[256];
    private final float[] floatPixels;
    private final int[] valueHistogram = new int[256];
    private final float inputScale, outputScale;
    private final int inputZero, outputZero;

    Classifier(AssetManager assets, boolean int8, int inputSize) throws IOException {
      byte[] bytes;
      String assetName = "ball_cnn_" + inputSize + (int8 ? "_int8.tflite" : "_fp32.tflite");
      try (InputStream stream = assets.open(assetName);
          ByteArrayOutputStream buffer = new ByteArrayOutputStream()) {
        byte[] chunk = new byte[16384];
        int n;
        while ((n = stream.read(chunk)) != -1) buffer.write(chunk, 0, n);
        bytes = buffer.toByteArray();
      }
      // TFLite needs native-order direct memory for model and tensor buffers.
      ByteBuffer weights = ByteBuffer.allocateDirect(bytes.length).order(ByteOrder.nativeOrder());
      weights.put(bytes);
      weights.rewind();
      model =
          new Interpreter(weights, new Interpreter.Options().setNumThreads(1).setUseXNNPACK(true));
      // Read tensor dimensions from the model instead of assuming a fixed crop size.
      int[] shape = model.getInputTensor(0).shape();
      size = shape[1];
      if (size != inputSize
          || !Arrays.equals(shape, new int[] {1, size, size, 3})
          || !Arrays.equals(model.getOutputTensor(0).shape(), new int[] {1, 4}))
        throw new IllegalArgumentException("Unexpected BIOBUZZ model tensor shape");
      cropPixels = new byte[size * size * 3];
      floatPixels = new float[cropPixels.length];
      // Use the model’s actual quantization scale/zero point, not hard-coded conversions.
      quantized = model.getInputTensor(0).dataType() == DataType.INT8;
      // Fail explicitly if a differently encoded model is accidentally packaged under this name.
      DataType expectedType = int8 ? DataType.INT8 : DataType.FLOAT32;
      if (model.getInputTensor(0).dataType() != expectedType
          || model.getOutputTensor(0).dataType() != expectedType) {
        model.close();
        throw new IllegalArgumentException("BIOBUZZ model precision does not match its selection");
      }
      inputScale = model.getInputTensor(0).quantizationParams().getScale();
      inputZero = model.getInputTensor(0).quantizationParams().getZeroPoint();
      outputScale = model.getOutputTensor(0).quantizationParams().getScale();
      outputZero = model.getOutputTensor(0).quantizationParams().getZeroPoint();
      input =
          ByteBuffer.allocateDirect(model.getInputTensor(0).numBytes())
              .order(ByteOrder.nativeOrder());
      output =
          ByteBuffer.allocateDirect(model.getOutputTensor(0).numBytes())
              .order(ByteOrder.nativeOrder());
    }

    // Pad one candidate, normalize its lighting, run the CNN, and return ball/background
    // confidence.
    private Score score(Mat rgb, Rect box) {
      long prepStart = System.nanoTime();
      // A small border preserves outline context while keeping adjacent objects mostly out.
      int pad = (int) (Math.max(box.width, box.height) * .08);
      Rect b =
          bounds(
              box.x - pad,
              box.y - pad,
              box.x + box.width + pad,
              box.y + box.height + pad,
              1,
              rgb.cols(),
              rgb.rows());
      Mat crop = rgb.submat(b);
      Imgproc.resize(crop, resizedCrop, new Size(size, size));
      crop.release();
      resizedCrop.get(0, 0, cropPixels);
      // Exact median of 8-bit V via a histogram, avoiding sort/float allocation.
      Arrays.fill(valueHistogram, 0);
      int margin = size / 4, half = (size - 2 * margin) * (size - 2 * margin) / 2;
      for (int y = margin; y < size - margin; y++)
        for (int x = margin; x < size - margin; x++) {
          int i = (y * size + x) * 3;
          valueHistogram[
              Math.max(
                  cropPixels[i] & 255,
                  Math.max(cropPixels[i + 1] & 255, cropPixels[i + 2] & 255))]++;
        }
      int cumulative = 0, lo = -1, hi = 0;
      for (int i = 0; i < 256; i++) {
        cumulative += valueHistogram[i];
        if (lo < 0 && cumulative > half - 1) lo = i;
        if (cumulative > half) {
          hi = i;
          break;
        }
      }
      // Brighten dark crops using their center brightness; do not darken already bright crops.
      double median = (lo + hi) / 510.,
          gamma =
              median > .015 && median < .5
                  ? Math.max(.55, Math.min(1, Math.log(.5) / Math.log(median)))
                  : 1;
      input.rewind();
      output.rewind();
      // Compute gamma and quantization once per possible byte, then apply the lookup to every
      // pixel.
      if (quantized) {
        for (int i = 0; i < 256; i++)
          quantizedLookup[i] =
              (byte)
                  Math.max(
                      -128,
                      Math.min(
                          127,
                          Math.round((float) Math.pow(i / 255., gamma) / inputScale) + inputZero));
        for (int i = 0; i < cropPixels.length; i++)
          cropPixels[i] = quantizedLookup[cropPixels[i] & 255];
        input.put(cropPixels);
      } else {
        float[] lut = new float[256];
        for (int i = 0; i < 256; i++) lut[i] = (float) Math.pow(i / 255., gamma);
        for (int i = 0; i < cropPixels.length; i++) floatPixels[i] = lut[cropPixels[i] & 255];
        input.asFloatBuffer().put(floatPixels);
      }
      input.rewind();
      double cropMs = (System.nanoTime() - prepStart) / 1e6;
      long runStart = System.nanoTime();
      // Only this native inference call contributes to modelMs.
      model.run(input, output);
      double modelMs = (System.nanoTime() - runStart) / 1e6;
      // The network returns logits; softmax converts them to relative class scores.
      output.rewind();
      double[] logits = new double[4];
      double max = -Double.MAX_VALUE, sum = 0;
      for (int i = 0; i < 4; i++) {
        logits[i] = quantized ? (output.get() - outputZero) * outputScale : output.getFloat();
        max = Math.max(max, logits[i]);
      }
      for (double value : logits) sum += Math.exp(value - max);
      // Ball confidence is 1 minus background score. HSV supplies the final color label.
      return new Score(1 - Math.exp(logits[0] - max) / sum, cropMs, modelMs);
    }

    @Override
    public void close() {
      model.close();
      resizedCrop.release();
    }
  }

  // Main pipeline: proposals → geometry/color checks → CNN → duplicate removal → XYZ.
  public List<Ball> detect(Mat rgb, double threshold, double fov) {
    trace.clear();
    lastModelMs = 0;
    lastCropMs = 0;
    lastInvocations = 0;
    long start = System.nanoTime();
    // Convert once per frame; both proposal generation and final color voting use this image.
    Mat hsv = new Mat();
    Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);
    // Yellow fallback needs boundary evidence before accepting an existing saturated body.
    buildEdges(rgb);
    List<Candidate> candidates = proposals(rgb, hsv);
    long postStart = System.nanoTime();
    List<Ball> found = new ArrayList<>(), kept = new ArrayList<>();
    Pixels hsvData = hsvPixels.read(hsv);
    buildColorSums(hsv);
    // On the Hub, do not pay for CNN inference on near-identical alternatives.
    // Keep the strongest geometric outline; retain separate touching objects.
    Map<Candidate, Double> qualities = new HashMap<>();
    Map<Candidate, Double> edges = new HashMap<>();
    for (Candidate c : candidates) {
      double edge = edgeSupport(c.box, c.cls == 1);
      if (traceEnabled)
        trace.add(
            "PROPOSAL cls="
                + c.cls
                + " box="
                + c.box
                + " edge="
                + edge
                + " color="
                + centralColor(c.box));
      edges.put(c, edge);
      // For yellow, favor round outlines over clipped patches or a ball merged with a wall.
      double roundness =
          Math.min(c.box.width, c.box.height) / (double) Math.max(c.box.width, c.box.height);
      qualities.put(
          c, quality(hsvData, c.box, c.cls) * (c.cls == 1 ? edge * Math.pow(roundness, 4) : 1));
    }
    // Yellow gets a stronger boundary requirement because pale walls can share its hue.
    candidates.removeIf(
        c -> edges.get(c) < (c.cls == 1 ? .40 : .25) || bodyCoverage(hsvData, c.box, c.cls) < .55);
    // Promote a body only over much smaller, enclosed fragments. A general size bonus would
    // prefer loose boxes over tighter ones and distort the size-based distance estimate.
    Map<Candidate, Double> ranks = new HashMap<>(qualities);
    candidates.sort(Comparator.comparingDouble(c -> c.box.area()));
    for (Candidate body : candidates)
      for (Candidate fragment : candidates)
        if (body.cls == fragment.cls
            && body.box.area() >= 3 * fragment.box.area()
            && intersection(body.box, fragment.box) > .85 * fragment.box.area())
          ranks.put(body, Math.max(ranks.get(body), ranks.get(fragment) + .000001));
    candidates.sort((a, b) -> Double.compare(ranks.get(b), ranks.get(a)));
    // Remove near-identical alternatives before CNN work while retaining distinct objects.
    List<Candidate> unique = new ArrayList<>();
    for (Candidate c : candidates) {
      boolean duplicate = false;
      for (Candidate p : unique)
        if (c.cls == p.cls && iou(c.box, p.box) > .8) {
          duplicate = true;
          break;
        }
      if (!duplicate) unique.add(c);
    }
    candidates = unique;
    lastPostMs = (System.nanoTime() - postStart) / 1e6;
    lastProposalMs = (System.nanoTime() - start) / 1e6;
    lastCandidateCount = candidates.size();
    start = System.nanoTime();
    // Reject boxes whose center color disagrees with the color that proposed them.
    List<Candidate> classified = new ArrayList<>();
    for (Candidate c : candidates) if (centralColor(c.box) == c.cls) classified.add(c);
    // Small parallel waves preserve quality order and skip later alternatives
    // only AFTER an overlapping candidate has actually passed the CNN.
    // Process a small batch, then use accepted balls to skip redundant later proposals.
    while (!classified.isEmpty()) {
      List<Candidate> wave = new ArrayList<>();
      List<Callable<Score>> jobs = new ArrayList<>();
      Iterator<Candidate> waiting = classified.iterator();
      while (waiting.hasNext() && wave.size() < classifiers.length) {
        Candidate c = waiting.next();
        if (covered(c, found)) {
          waiting.remove();
          continue;
        }
        // Defer overlapping alternatives while their stronger outline is being checked.
        // If that outline fails, the deferred candidate remains available next wave.
        boolean inFlight = false;
        for (Candidate other : wave) {
          if (c.cls == other.cls && sameOutline(c.box, other.box)) {
            inFlight = true;
            break;
          }
        }
        if (inFlight) continue;
        waiting.remove();
        Classifier classifier = classifiers[wave.size()];
        wave.add(c);
        jobs.add(() -> classifier.score(rgb, c.box));
      }
      List<Score> scores = runAll(jobs);
      for (int i = 0; i < wave.size(); i++) {
        Candidate c = wave.get(i);
        Score evaluated = scores.get(i);
        if (traceEnabled)
          trace.add("CNN cls=" + c.cls + " box=" + c.box + " confidence=" + evaluated.confidence);
        lastCropMs += evaluated.cropMs;
        lastModelMs += evaluated.modelMs;
        lastInvocations++;
        if (!covered(c, found) && evaluated.confidence >= threshold)
          found.add(
              new Ball(
                  c.box,
                  c.cls,
                  evaluated.confidence,
                  qualities.get(c),
                  rgb.cols(),
                  rgb.rows(),
                  fov));
      }
    }
    lastInferenceMs = (System.nanoTime() - start) / 1e6;
    hsv.release();
    // Prefer high confidence and strong geometry when two accepted outlines overlap.
    found.sort((a, b) -> Double.compare(b.score * b.quality, a.score * a.quality));
    for (Ball b : found) {
      boolean keep = true;
      for (Ball k : kept)
        if (b.cls == k.cls && sameOutline(b.box, k.box)) {
          keep = false;
          break;
        }
      if (keep) kept.add(b);
    }
    return Collections.unmodifiableList(kept);
  }

  // Suppress same-color duplicates if IoU ≥ 45% or over 80% of the smaller box is covered.
  private static boolean covered(Candidate c, List<Ball> found) {
    for (Ball b : found) if (c.cls == b.cls && sameOutline(c.box, b.box)) return true;
    return false;
  }

  // Use one overlap rule for deferring work and suppressing accepted duplicates.
  private static boolean sameOutline(Rect a, Rect b) {
    return iou(a, b) >= .45 || intersection(a, b) / Math.min(a.area(), b.area()) > .80;
  }

  @Override
  public void close() {
    // The owning frame worker calls close only after all detect work has returned.
    if (pool != null) pool.shutdown();
    for (Classifier c : classifiers) c.close();
    openKernel.release();
    closeKernel.release();
    gray.release();
    gradientX.release();
    gradientY.release();
  }
}
