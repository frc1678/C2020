package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.lib.control.PIDController;

import edu.wpi.first.wpilibj.AnalogInput;

public class WheelDrive {
    private TalonFX angleMotor;
    private TalonFX speedMotor;
    private PIDController pidController;

    /*
     * @param angleMotor the port for the angle motor
     * 
     * @param speedMotor the port for the speed motor
     * 
     * @param encoder the port for the encoder
     */
    public WheelDrive(int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new TalonFX(angleMotor);
        this.speedMotor = new TalonFX(speedMotor);
        //pidController = new PIDController(1, 0, 0, new AnalogInput(encoder), this.angleMotor);
        //pidController.setContinuous();
        //pidController.enable();
    }

    private final double MAX_VOLTS = 4.95; // Maximum voltage taken by swerve module.
    /*
     * @param speed the speed of the wheels
     * 
     * @param angle the angle of the wheels
     */

    public void drive(double speed, double angle) {
        speedMotor.set(ControlMode.PercentOutput, speed);

        double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = MAX_VOLTS + setpoint;
        }
        if (setpoint > MAX_VOLTS) {
            setpoint = setpoint - MAX_VOLTS;
        }
        //Set Goal is our version of "setpoint"
        pidController.setGoal(setpoint);
    }
}