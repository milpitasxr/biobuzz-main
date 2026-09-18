package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double Kp, Ki, Kd;
    private double integral = 0;
    private double lastError = 0;
    private double lastDerivative = 0;
    private double maxIntegral;
    public double alpha; // derivative low-pass factor: 0 = no filter, 0.2 typical
    private ElapsedTime timer = new ElapsedTime();

    // Constructor with default alpha = 0 (no filter)
    public PIDController(double Kp, double Ki, double Kd, double maxIntegral){
        this(Kp, Ki, Kd, maxIntegral, 0.0);
    }

    // Constructor with custom alpha
    public PIDController(double Kp, double Ki, double Kd, double maxIntegral, double alpha){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.maxIntegral = maxIntegral;
        this.alpha = alpha;
        timer.reset();
    }

    // Update all constants dynamically
    public void setConstants(double Kp, double Ki, double Kd, double maxIntegral){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.maxIntegral = maxIntegral;
    }

    // Optional: reset internal state
    public void reset(){
        integral = 0;
        lastError = 0;
        lastDerivative = 0;
        timer.reset();
    }

    // Compute PID output
    public double update(double error){
        double dt = timer.seconds();
        timer.reset();
        if(dt <= 0){
            dt = 1e-4;
        }

        // --- Integral with anti-windup ---
        integral += error * dt;
        if(integral > maxIntegral){
            integral = maxIntegral;
        }
        if(integral < -maxIntegral) {
            integral = -maxIntegral;
        }

        // --- Derivative with optional low-pass filter ---
        double derivativeRaw = (error - lastError) / dt;
        lastError = error;

        double derivative = derivativeRaw;
        if(alpha > 0){
            derivative = alpha * lastDerivative + (1 - alpha) * derivativeRaw;
            lastDerivative = derivative;
        }

        // --- PID output ---
        return Kp * error + Ki * integral + Kd * derivative;
    }
}