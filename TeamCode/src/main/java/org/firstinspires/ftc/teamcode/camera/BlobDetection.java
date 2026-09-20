package org.firstinspires.ftc.teamcode.camera;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.teamcode.Alliance;
import java.util.List;
@TeleOp(name="Blob detection", group="vision")
public class BlobDetection extends LinearOpMode{
    private static final String CAMERA_NAME = "Webcam 1";
    private static final int FRAME_WIDTH  = 640; // getting from docs
    private static final int FRAME_HEIGHT = 480; // getting from docs
    private static final double FOV_DEG    = 76.0; // getting from docs
    private static final double FOCAL_PX = (FRAME_WIDTH/2)/Math.tan(FOV_DEG/2); // getting the focal px which is used to actually turn the camera into smth real
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // EDIT IN COMP
    //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    private static final int EXPOS_MS = 6; // how dark it looks greater value means that brighter but more motion blur, however shorter is crispier but darker
    private static final int GAIN = 100; // increases brigthness but in result it adds noise, you can use this to make it more bright if the expos_ms is too low which allows for the crisp, but too much and itll add hella noise
    private static final int WHITE_BALENCE = 4000; // this is what white should look like gym leds are around this but if purple doesnt look purple in stream change this value
    private static final double FOCUS_VAL = 0; // this basically means focus on everything nothing in specific
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // OFFSETS FROM CENTER, MUST BE TUNED
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    private static final double CAM_OFFSET_X = 7.0; // must be tuned
    private static final double CAM_OFFSET_Y = -2.0; // must be tuned
    private static final double CAM_YAW_DEG = 0.0; // must be tuned basically where the camera points realitive to the robots forward. NEG IF RIGHT
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // FILTERING THE BLOB TO MAKE SURE IT ACTUALLY THERE
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    private static final double MIN_RADIUS_PX = 4.0; // any smaller then this then it gets to untrust worthy
    private static final double MAX_RADIUS_PX = 150; // changeable any greater and well not trustworthy
    private static final double MAX_AREA_PX = 80000; // This is just to clear it and make sure that the blob is a good size
    private static final double MIN_CIRCULARITY = 0.4; // this is like the minimum circle the blob must detect
    private ColorBlobLocatorProcessor purpleLocator;
    private VisionPortal portal;
    @Override
    public void runOpMode() {
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY) // this makes it so that we only care about the color
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75)) // thisd is basically just cropping the camerda to ignore the static on the last perdcent of the camerad
                .setDrawContours(true) // paints an outline of the blob so we can see it
                .setBoxFitColor(0) // basically the sa e thing but is a box, setting it to 0 means don't box fit
                .setCircleFitColor(Color.rgb(255, 0, 255))// adding a circle around it instead of a rectangle
                .setBlurSize(5) // cleans the blur out
                .setDilateSize(15) // basically used to hide the holes and etc
                .setErodeSize(15) // restoring it back to its original size after covering the holes with the dilate size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING) // we are dilating first meaning we are covering the hole first then we are removing the extra pixels added because of the dilation
                .build(); // building my classifier
        ColorBlobLocatorProcessor nectar = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange((Alliance.current == Alliance.BLUE) ? ColorRange.BLUE : ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY) // this makes it so that we don't carea about holes inside the blob, just the outline
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75)) // thisd is basically just cropping the camerda to ignore the static on the last perdcent of the camerad
                .setDrawContours(true) // paints an outline of the blob so we can see it
                .setBoxFitColor(0) // basically the sa e thing but is a box, setting it to 0 means don't box fit
                .setCircleFitColor(Color.rgb(255, 0, 255))// adding a circle around it instead of a rectangle
                .setBlurSize(5) // smooths image so there isnt much noise
                .setDilateSize(15) // basically used to hide the holes and etc
                .setErodeSize(15) // restoring it back to its original size after covering the holes with the dilate size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING) // we are dilating first meaning we are covering the hole first then we are removing the extra pixels added because of the dilation
                .build(); // building my classifier
        WebcamName webcam = hardwareMap.get(WebcamName.class, CAMERA_NAME);
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(FRAME_WIDTH, FRAME_HEIGHT))
                .addProcessors(colorLocator, nectar)
                .build();

    }



}

/// 1. Auton, Teleop, Camera, Shooter, Intake
