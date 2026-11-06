package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

/**
 * Field-centric mecanum drive. The goBILDA Pinpoint is the localizer: it gives the robot's
 * field pose (x, y in inches, heading in radians), configured from Constants.localizerConfig
 * so TeleOp and Pedro Auto share the exact same pod offsets/directions.
 */
@Configurable
public class Drivetrain {
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    OpMode op;
    public double powerFactor = 1;
    Gamepad xbox;

    PinpointLocalizer localizer;
    Pose pose = new Pose(0, 0, 0);   // field pose, read from the Pinpoint once per loop

    // Field heading (radians) that points straight away from the driver.
    // Stick "up" drives the robot toward this heading. Back button re-zeroes it.
    double driverForward;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // Tune these for your bot
    static final double DRIVE_DEADZONE = 0.05;      // ignore tiny stick drift
    static final double TURN_FACTOR = 0.6;   // how fast right stick spins the bot

    public Drivetrain(OpMode opmode) {
        // Initialize variables
        op = opmode;
        xbox = op.gamepad1;
        rightFront = op.hardwareMap.get(DcMotorEx.class, "fR");
        leftFront = op.hardwareMap.get(DcMotorEx.class, "fL");
        leftBack = op.hardwareMap.get(DcMotorEx.class, "bL");
        rightBack = op.hardwareMap.get(DcMotorEx.class, "bR");

        // Pinpoint setup comes from Pedro's config (recalibrates the IMU, keep the robot still in init)
        localizer = new PinpointLocalizer(op.hardwareMap, Constants.localizerConfig);
        // Pick up where Auto left off so the field pose stays continuous into TeleOp
        localizer.setPose(PoseStorage.currentPose);
        pose = PoseStorage.currentPose;

        // Whatever way the robot faces at init is "forward" for the driver
        driverForward = pose.heading();

        // Set Zero power behavior for all 4 drive motors
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reverse direction of motors if needed (prolly left motors icl)
        // leftFront.setDirection(DcMotorEx.Direction.REVERSE);
    }

    /** Call this once per loop. Updates the Pinpoint, then drives field-centric from both sticks. */
    public void update() {
        localizer.update();
        pose = localizer.state().pose();
        PoseStorage.currentPose = pose;

        // Back button: "forward" becomes wherever the robot is facing right now.
        // Only the driver offset changes; the Pinpoint's field pose is left alone.
        if (xbox.back) {
            driverForward = pose.heading();
        }

        // Robot heading measured from the driver's forward direction
        double heading = AngleUnit.normalizeRadians(pose.heading() - driverForward);

        // Left stick as a vector: x = right, y = forward (stick y is inverted)
        double x = deadzone(xbox.left_stick_x);
        double y = deadzone(-xbox.left_stick_y);

        double[] robotVector = fieldToRobot(x, y, heading);
        double x1 = robotVector[0];
        double y1 = robotVector[1];

        double power = Math.hypot(x1, y1);
        double theta = Math.atan2(y1, x1);
        double turn = getTurn();

        panelsTelemetry.addData("X (in)", pose.x());
        panelsTelemetry.addData("Y (in)", pose.y());
        panelsTelemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        panelsTelemetry.addData("Driver heading (deg)", Math.toDegrees(heading));

        updateDrivetrain(turn, theta, power);
    }

    /** Field pose from the Pinpoint (inches, radians). */
    public Pose getPose() {
        return pose;
    }

    /**
     * Rotates a field-frame stick vector into the robot's frame using a 2D rotation matrix.
     * heading is the robot's yaw in radians (counter-clockwise positive).
     * Returns {x, y} in the robot frame.
     */
    private double[] fieldToRobot(double x, double y, double heading) {
        // Rotate by -heading: [cos h  sin h; -sin h  cos h] * [x; y]
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double robotX = x * cos + y * sin;
        double robotY = -x * sin + y * cos;
        return new double[]{robotX, robotY};
    }

    private double deadzone(double value) {
        return Math.abs(value) > DRIVE_DEADZONE ? value : 0;
    }

    private double getTurn() {
        // Right stick drives rotation directly, scaled down so it isn't too twitchy
        return deadzone(xbox.right_stick_x) * TURN_FACTOR;
    }

    public void updateDrivetrain(double turn, double theta, double power) {
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = power * cos / max + turn;
        double rightFrontPower = power * sin / max - turn;
        double leftBackPower = power * sin / max + turn;
        double rightBackPower = power * cos / max - turn;

        double maxPower = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                        Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
                )
        );

        leftFrontPower /= maxPower;
        rightFrontPower /= maxPower;
        leftBackPower /= maxPower;
        rightBackPower /= maxPower;

        leftFront.setPower(-leftFrontPower * powerFactor);
        rightFront.setPower(rightFrontPower * powerFactor);
        leftBack.setPower(-leftBackPower * powerFactor);
        rightBack.setPower(rightBackPower * powerFactor);
    }
}
