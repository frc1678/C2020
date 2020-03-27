package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;

public class WheelDrive extends Subsystem {
    private TalonFX angleMotor;
    private TalonFX speedMotor;
    private PIDController pidController;

    private final double kMaxVolts = 4.95;

    public WheelDrive(int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new TalonFX(angleMotor);
        this.speedMotor = new TalonFX(speedMotor);
        pidController = new PIDController(1, 0, 0/*, new AnalogInput(encoder), this.angleMotor*/);

        //pidController.setOutputRange(-1, 1);
        //pidController.setContinuous();
        //pidController.enable();
    }

    public void drive (double speed, double angle) {
        //speedMotor.set (speed);
    
        double setpoint = angle * (kMaxVolts * 0.5) + (kMaxVolts * 0.5); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = kMaxVolts + setpoint;
        }
        if (setpoint > kMaxVolts) {
            setpoint = setpoint - kMaxVolts;
        }
    
        pidController.setSetpoint (setpoint);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }

}