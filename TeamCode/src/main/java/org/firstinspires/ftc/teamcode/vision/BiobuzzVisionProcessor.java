package org.firstinspires.ftc.teamcode.vision;

import android.content.res.AssetManager;
import android.graphics.*;
import java.io.IOException;
import java.util.*;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/** A single-slot worker consumes newest frames; capture/preview never waits for CNN inference. */
public final class BiobuzzVisionProcessor
    implements VisionProcessor, CameraStreamSource, AutoCloseable {
  // Publish detections and their measurements together, so readers see one consistent frame.
  public static final class Result {
    public final List<BiobuzzDetector.Ball> balls;
    // Original camera capture time, not the time inference finished.
    public final long timestamp;
    public final double milliseconds;
    // Includes time between camera capture and publishing this result, not browser/network delay.
    public final double captureToResultMs;
    public final String error;
    public final double proposalMs, cropMs, modelMs;
    public final String stages;
    public final int candidates, invocations;

    Result(List<BiobuzzDetector.Ball> b, long t, double ms, String e, BiobuzzDetector d) {
      balls = b;
      timestamp = t;
      milliseconds = ms;
      captureToResultMs = t == 0 ? 0 : (System.nanoTime() - t) / 1e6;
      error = e;
      proposalMs = d == null ? 0 : d.lastProposalMs;
      cropMs = d == null ? 0 : d.lastCropMs;
      modelMs = d == null ? 0 : d.lastModelMs;
      candidates = d == null ? 0 : d.lastCandidateCount;
      invocations = d == null ? 0 : d.lastInvocations;
      stages =
          d == null
              ? ""
              : "split="
                  + Arrays.toString(d.splitMs)
                  + " outline="
                  + Arrays.toString(d.outlineMs)
                  + " post="
                  + d.lastPostMs;
    }

    // Age includes waiting and processing; this is different from detector execution time.
    public double ageMs() {
      return timestamp == 0 ? Double.POSITIVE_INFINITY : (System.nanoTime() - timestamp) / 1e6;
    }
  }

  private final BiobuzzDetector detector;
  private final int previousOpenCvThreads;
  // Volatile settings allow Dashboard changes to reach the inference worker safely.
  public volatile double threshold = .85, horizontalFov = 70;
  public volatile int proposalWidth = 800;
  // Fraction of wall time this one worker may spend processing rather than deliberately idling.
  // This is a detector budget, not a hard cap on the whole Hub or the other SDK threads.
  public volatile double dutyPercent = 100;
  public volatile double pacingIdleMs = 0;
  // Replacing this reference publishes the entire completed result atomically.
  public volatile Result latest = new Result(Collections.emptyList(), 0, 0, "", null);
  // Publish a new bitmap rather than modifying one a streaming client might still read.
  private volatile Bitmap preview = Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565);
  private long lastPreview = 0;
  private volatile boolean closed = false;
  // This lock protects ownership of the waiting frame and spare image buffer.
  private final Object pendingLock = new Object();
  // At most one frame waits; spare is reusable storage that no worker is currently reading.
  private Mat pending, spare;
  private long pendingTimestamp;
  private final Thread worker;
  // Count replaced waiting frames, not camera failures. Dropping backlog keeps results fresh.
  public volatile int droppedFrames = 0;
  public volatile double detectionFps = 0;

  public BiobuzzVisionProcessor(AssetManager assets, boolean int8, int threads) throws IOException {
    this(assets, int8, threads, 96);
  }

  public BiobuzzVisionProcessor(AssetManager assets, boolean int8, int threads, int inputSize)
      throws IOException {
    // OpenCV stays single-threaded, including when proposal jobs run sequentially.
    previousOpenCvThreads = Core.getNumThreads();
    Core.setNumThreads(1);
    try {
      detector = new BiobuzzDetector(assets, int8, threads, inputSize);
    } catch (IOException | RuntimeException e) {
      Core.setNumThreads(previousOpenCvThreads);
      throw e;
    }
    worker = new Thread(this::runWorker, "BIOBUZZ-inference");
    worker.setDaemon(true);
    worker.start();
  }

  @Override
  public void init(int width, int height, CameraCalibration calibration) {}

  // Only this thread owns detector lifetime and invokes detect.
  private void runWorker() {
    long previous = 0;
    long previousFinished = 0, previousWork = 0;
    try {
      while (!closed) {
        Mat rgb;
        long stamp;
        // Take ownership under the lock, then do expensive inference outside it.
        synchronized (pendingLock) {
          while (!closed) {
            if (pending == null) {
              pendingLock.wait();
              continue;
            }
            // Charge the completed job for its measured duration: a longer job earns more idle.
            // Live changes take effect here without rebuilding the model or restarting capture.
            double budget =
                Double.isFinite(dutyPercent) ? Math.max(10, Math.min(100, dutyPercent)) : 100;
            long idleNanos = (long) (previousWork * (100 / budget - 1));
            pacingIdleMs = idleNanos / 1e6;
            long remaining = previousFinished + idleNanos - System.nanoTime();
            if (remaining <= 0) break;
            // wait releases the lock, so incoming frames replace the pending image throughout
            // the pause. Never take a frame first and then sleep with an aging image in hand.
            long waitNanos = Math.min(remaining, 50_000_000L);
            pendingLock.wait(waitNanos / 1_000_000L, (int) (waitNanos % 1_000_000L));
          }
          if (closed) break;
          rgb = pending;
          pending = null;
          stamp = pendingTimestamp;
        }
        long start = System.nanoTime();
        try {
          detector.proposalWidth = proposalWidth;
          List<BiobuzzDetector.Ball> balls =
              detector.detect(
                  rgb,
                  Math.max(.3, Math.min(1, threshold)),
                  Math.max(20, Math.min(150, horizontalFov)));
          long now = System.nanoTime();
          // Publish a complete result in one reference update.
          latest = new Result(balls, stamp, (now - start) / 1e6, "", detector);
          if (previous != 0) detectionFps = 1e9 / (now - previous);
          previous = now;
        } catch (Exception e) {
          // Publish a complete result in one reference update.
          latest = new Result(Collections.emptyList(), stamp, 0, e.toString(), detector);
        } finally {
          synchronized (pendingLock) {
            if (!closed && spare == null) spare = rgb;
            else rgb.release();
          }
          // Include cleanup in the budget. close() can wake a paced worker immediately.
          previousFinished = System.nanoTime();
          previousWork = previousFinished - start;
        }
      }
    } catch (InterruptedException ignored) {
      Thread.currentThread().interrupt();
    } finally {
      synchronized (pendingLock) {
        if (pending != null) {
          pending.release();
          pending = null;
        }
        if (spare != null) {
          spare.release();
          spare = null;
        }
      }
      detector.close();
      Core.setNumThreads(previousOpenCvThreads);
    }
  }

  @Override
  public Object processFrame(Mat frame, long captureTimeNanos) {
    if (closed) return latest;
    Result result = latest;
    // Give inference its frame before spending CPU time drawing the preview.
    synchronized (pendingLock) {
      if (closed) return result;
      // Overwrite the waiting frame in place. The worker's active frame
      // is separate, so capture cannot modify an image under inference.
      if (pending != null) droppedFrames++;
      else if (spare != null) {
        pending = spare;
        spare = null;
      } else pending = new Mat();
      if (frame.channels() == 4) Imgproc.cvtColor(frame, pending, Imgproc.COLOR_RGBA2RGB);
      else frame.copyTo(pending);
      // Keep the image and its capture timestamp paired when replacing a pending frame.
      pendingTimestamp = captureTimeNanos;
      pendingLock.notifyAll();
    }
    // Preview uses the CURRENT frame; old detections are explicitly age-labeled.
    // Limit preview conversion/drawing to 10 FPS while capture can continue at 30 FPS.
    if (System.nanoTime() - lastPreview > 100_000_000L) {
      Bitmap bitmap = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
      Utils.matToBitmap(frame, bitmap);
      drawResult(new Canvas(bitmap), result, 1, 1);
      preview = bitmap;
      lastPreview = System.nanoTime();
    }
    return result;
  }

  // Hide stale/error results, and label how old the overlaid detections are.
  private static void drawResult(Canvas canvas, Result result, float scale, float density) {
    double age = result.ageMs();
    if (age < 1500 && result.error.isEmpty()) draw(canvas, result.balls, scale, density);
    Paint p = new Paint(Paint.ANTI_ALIAS_FLAG);
    p.setTextSize(14 * density);
    p.setColor(Color.argb(200, 0, 0, 0));
    canvas.drawRect(0, 0, canvas.getWidth(), 22 * density, p);
    p.setColor(Color.WHITE);
    String status =
        age < 1500 && result.error.isEmpty()
            ? String.format(Locale.US, "LIVE preview | detections %.0f ms old", age)
            : "LIVE preview | waiting for fresh detections";
    canvas.drawText(status, 5 * density, 16 * density, p);
  }

  // Draw camera-coordinate boxes and readable labels on the preview image.
  private static void draw(
      Canvas canvas, List<BiobuzzDetector.Ball> balls, float scale, float density) {
    Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    paint.setStrokeWidth(2 * density);
    paint.setTextSize(15 * density);
    int n = 0;
    int[] colors = {
      Color.GRAY, Color.rgb(255, 215, 55), Color.rgb(255, 70, 100), Color.rgb(65, 155, 255)
    };
    for (BiobuzzDetector.Ball b : balls) {
      n++;
      paint.setColor(colors[b.cls]);
      paint.setStyle(Paint.Style.STROKE);
      float x = b.box.x * scale, y = b.box.y * scale;
      canvas.drawRect(
          x, y, (b.box.x + b.box.width) * scale, (b.box.y + b.box.height) * scale, paint);
      paint.setStyle(Paint.Style.FILL);
      String label =
          String.format(
              Locale.US,
              "#%d %s %.0f%% Z %.2fm",
              n,
              BiobuzzDetector.LABELS[b.cls],
              b.score * 100,
              b.z);
      // Clamp label placement so text stays inside the preview where possible.
      float tx = Math.max(0, Math.min(x, canvas.getWidth() - paint.measureText(label) - 6)),
          ty = Math.max(18 * density, y - 4);
      canvas.drawRect(tx, ty - 16 * density, tx + paint.measureText(label) + 6, ty + 4, paint);
      paint.setColor(Color.BLACK);
      canvas.drawText(label, tx + 3, ty, paint);
    }
  }

  @Override
  public void onDrawFrame(
      Canvas canvas, int width, int height, float scale, float density, Object context) {
    if (context instanceof Result) drawResult(canvas, (Result) context, scale, density);
  }

  @Override
  public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
    // Capture one immutable published reference before dispatching to the stream consumer.
    final Bitmap image = preview;
    continuation.dispatch(consumer -> consumer.accept(image));
  }

  @Override
  public void close() {
    // Wake the sleeping worker and let it release its own interpreter safely.
    closed = true;
    synchronized (pendingLock) {
      pendingLock.notifyAll();
    }
    try {
      worker.join(1000);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
    // Worker owns and closes the interpreter, avoiding a close/invoke race.
  }
}
