package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.PIDController;

public class Drivetrain {
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    OpMode op;
    public double powerFactor = 1;
    Gamepad xbox;

    IMU imu;
    PIDController drivePID;
    double targetHeading = 0; // radians, updated whenever we're not manually turning

    // Tune these for your bot
    static final double TURN_DEADZONE = 0.05;      // ignore tiny stick drift
    static final double MANUAL_TURN_SCALE = 0.6;   // how fast right stick spins the bot
    static final double MAX_HEADING_CORRECTION = 0.5; // clamp PID output so it can't fight the drivetrain too hard

    // PID Values
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double maxIntegral = 0;

    public Drivetrain(OpMode opmode, IMU imu1) {
        // Initialize variables
        op = opmode;
        xbox = op.gamepad1;
        rightFront = op.hardwareMap.get(DcMotorEx.class, "rF");
        leftFront = op.hardwareMap.get(DcMotorEx.class, "lF");
        leftBack = op.hardwareMap.get(DcMotorEx.class, "lB");
        rightBack = op.hardwareMap.get(DcMotorEx.class, "rB");

        imu = imu1;
        imu.resetYaw();

        // Kp, Ki, Kd, maxIntegral — start with just Kp and tune from there
        drivePID = new PIDController(0.8, 0.0, 0.05, 0.3);
        targetHeading = getHeading();

        // Set Zero power behavior for all 4 drive motors
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reverse direction of motors if needed (prolly left motors icl)
        // leftFront.setDirection(DcMotorEx.Direction.REVERSE);


    }

    /** Call this once per loop. Reads both sticks and drives the robot with heading lock. */
    public void update() {
        double power = getPower();
        double theta = getTheta();
        double turn = getTurn();

        updateDrivetrain(turn, theta, power);
    }

    // Get length of hypotenuse
    public double getPower() {
        double x = xbox.left_stick_x;
        double y = -xbox.left_stick_y;
        return Math.hypot(x, y);
    }

    // Get the theta of that the left joystick creates
    public double getTheta() {
        double x = xbox.left_stick_x;
        double y = -xbox.left_stick_y;
        return Math.atan2(y, x);
    }

    private double getTurn() {
        // Get turn and current heading
        double rightStickX = xbox.right_stick_x;
        double currentHeading = getHeading();

        // Checks to make sure joystick is acutally pushed
        if (Math.abs(rightStickX) > TURN_DEADZONE) {
            // Manual turning: drive rotation directly from the stick,
            // and keep re-marking the target heading so it locks wherever you let go.
            targetHeading = currentHeading;
            drivePID.reset(); // avoid an integral/derivative jump when we re-engage the lock
            return rightStickX * MANUAL_TURN_SCALE; // takes only a factor of the actual turn

        } else {
            // Locked: hold targetHeading using PID
            double error = angleWrap(targetHeading - currentHeading);
            double correction = drivePID.update(error);

            // Clamp so heading-hold can't overpower translation
            if (correction > MAX_HEADING_CORRECTION) correction = MAX_HEADING_CORRECTION;
            if (correction < -MAX_HEADING_CORRECTION) correction = -MAX_HEADING_CORRECTION;

            return correction;
        }
    }

    // Get current heading of the robot using IMU
    private double getHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    // Normalizes angle to between [-PI, PI]
    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
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
        rightFront.setPower(-rightFrontPower * powerFactor);
        leftBack.setPower(-leftBackPower * powerFactor);
        rightBack.setPower(-rightBackPower * powerFactor);
    }


}