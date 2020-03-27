package com.team1678.frc2020.subsystems;

import com.team1678.lib.control.PIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class WheelDrive extends Subsystem{
    private TalonFX motorSpeed;
    private PIDController pidController;

    private final double maxV = 4.95; // MAX voltage

    public void drive(double speed, double angle) {

        motorSpeed.set(ControlMode.PercentOutput, speed); // motorSpeed == speedMotor

        double setpoint = angle * (maxV * 0.5) + (maxV * 0.5); // Calculate offset
        if (setpoint < 0) {
            setpoint = maxV + setpoint;
        }
        if (setpoint > maxV) {
            setpoint = setpoint - maxV;
        }
        pidController.setGoal(setpoint); // setGoal == setPoint
    }

    public WheelDrive(int motorAngle, int motorSpeed, int encoder) {
        new TalonFX(motorAngle);
        this.motorSpeed = new TalonFX(motorSpeed);
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }
}