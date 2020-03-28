package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.lib.control.PIDController;

//import edu.wpi.first.wpilibj.AnalogInput;

public class WheelDrive extends Subsystem {
    private TalonFX mAngleMotor;
    private TalonFX mSpeedMotor;
    private PIDController mPIDController;

    public WheelDrive(int angleMotor, int speedMotor, int encoder) {
        this.mAngleMotor = new TalonFX(angleMotor);
        this.mSpeedMotor = new TalonFX(speedMotor);
        mPIDController = new PIDController(1, 0, 0/* ,new AnalogInput (encoder), this.mAngleMotor */);

        // mPIDController.setOutputRange (-1, 1);
        // mPIDController.setContinuous ();
        // mPIDController.enable ();
    }

    private final double kMaxVolts = 4.95;

    public void drive(double speed, double angle) {
        mSpeedMotor.set(ControlMode.PercentOutput, speed);

        double setpoint = angle * (kMaxVolts * 0.5) + (kMaxVolts * 0.5);
        if (setpoint < 0) {
            setpoint = kMaxVolts + setpoint;
        }
        if (setpoint > kMaxVolts) {
            setpoint = setpoint - kMaxVolts;
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