package com.team1678.lib.control;

/**
 * PID controller that controls a control loop
 */


public class PIDController {

    private double kP;
    private double kI;
    private double kD;
    private double setpoint = 0.0;
 
    private double prevError = 0.0;
    private double integral = 0.0;

    private double prevTime = 0.0;
    private double kDeadband = 0.0;

    //added from edu.wpi.first.wpilibj.PIDController
    private double m_maximumOutput = 1.0;	// |maximum output|
    private double m_minimumOutput = -1.0;	// |minimum output|
    private boolean m_continuous = false;	// do the endpoints wrap around? eg. Absolute encoder
    private boolean m_enabled = false; 		//is the pid controller enabled
    

    
    public PIDController(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    public PIDController(double p, double i, double d, double deadband) {
        kP = p;
        kI = i;
        kD = d;
        kDeadband = deadband;
    }

    public void reset() {
        prevError = 0;
        prevTime = 0;
        integral = 0;

    }

    public double update(double timestamp, double sensor) {
        double dt = timestamp - prevTime;
        
        double error = setpoint - sensor;

        prevTime = timestamp;

        if (Math.abs(error) < kDeadband) {
            return 0.0;
        }
        return (kP * error) + (kI * calculateIntegral(dt, error)) + (kD * calculateDerivative(dt, error));
    }

    public void setGoal(double setpoint) {
        this.setpoint = setpoint;
    }

    // calculate functions could have been in the update lol

    private double calculateDerivative(double dt, double error) {
        double derivative = (error - prevError) / dt;
        prevError = error;
        return derivative;
    }

    private double calculateIntegral(double dt, double error) {
        integral += error * dt;
      //  prevError = error;
        return integral;
    }

    //added from edu.wpi.first.wpilibj.PIDController
    public synchronized void setOutputRange(double minimumOutput, double maximumOutput) {
        if (minimumOutput > maximumOutput) {
            //throw new BoundaryException("Lower bound is greater than upper bound");
        }
        m_minimumOutput = minimumOutput;
        m_maximumOutput = maximumOutput;
    }

    public synchronized void setContinuous(boolean continuous) {
        m_continuous = continuous;
    }

    public synchronized void enable() {
        m_enabled = true;

        // Removed
        //if (table != null) {
        //    table.putBoolean("enabled", true);
        }
    }
}