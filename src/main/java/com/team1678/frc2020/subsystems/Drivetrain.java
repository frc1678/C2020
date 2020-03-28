package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.lib.control.PIDController;

public class Drivetrain extends Subsystem {
    private TalonFX mAngleMotor;
    private TalonFX mSpeedMotor;
    private PIDController mPIDController;

    private final static double kMaxVoltage = 4.95; // max voltage for a swerve module

    public Drivetrain(int angleMotor, int speedMotor, int encoder) {
        this.mAngleMotor = new TalonFX(angleMotor);
        this.mSpeedMotor = new TalonFX(speedMotor);

        mPIDController = new PIDController(1, 0, 0/*, new AnalogInput (encoder), this.angleMotor*/);
        // mPIDController.setContinuous();
        // mPIDController.enable();
        // mPIDController.setOutputRange(-1, 1);
    }

    public void drive(double speed, double angle) {
        mSpeedMotor.set(ControlMode.PercentOutput,speed);
    
        double setpoint = angle * (kMaxVoltage * 0.5) + (kMaxVoltage * 0.5); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = kMaxVoltage + setpoint;
        }
        if (setpoint > kMaxVoltage) {
            setpoint = setpoint - kMaxVoltage;
        }
    
        mPIDController.setGoal(setpoint);
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
