package com.team1678.frc2020.subsystems;

import com.team1678.lib.control.PIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Talon;

public class Drivetrain {
    private Talon angleMotor;
    private Talon speedMotor;
    private PIDController pidController;

    private final double MAX_VOLTS = 4.95;

    public Drivetrain (int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new Talon (angleMotor);
        this.speedMotor = new Talon (speedMotor);
        pidController = new PIDController (1, 0, 0, new AnalogInput (encoder), this.angleMotor);
    
        /*pidController.setOutputRange (-1, 1);
        pidController.setContinuous ();
        pidController.enable ();*/
        // commented out section was giving errors
    }

    public Drivetrain(Drivetrain backLeft, Drivetrain backRight, Drivetrain frontLeft, Drivetrain frontRight) {
	}

	public void drive (double speed, double angle) {
        speedMotor.set (speed);
    
        double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = MAX_VOLTS + setpoint;
        }
        if (setpoint > MAX_VOLTS) {
            setpoint = setpoint - MAX_VOLTS;
        }
    
        pidController.setGoal(setpoint);
    }
}
