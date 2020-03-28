package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.lib.control.PIDController;

public class SwerveModule extends Subsystem {
    private final double kMaximumVoltage = 4; // I don't know what a reasonable maximum voltage would be

    private final TalonFX mDriveMotor;
    private final TalonFX mRotationMotor;

    private final PIDController mPidController;

    private static PeriodicIO mPeriodicIO = new PeriodicIO();

    public SwerveModule(int driveMotorPort, int rotationMotorPort, int encoderPort) {
        mDriveMotor = new TalonFX(driveMotorPort);
        mRotationMotor = new TalonFX(driveMotorPort);

        // I had problems with the PID controller from the Gitbook document, so I decided to use the one we have. This would most likely not work
        mPidController = new PIDController (1, 0, 0);

        /* This is what was supposed to be here, but there were multiple errors
            ------------------------------------------------------------------
            pidController = new PIDController(1, 0, 0, new AnalogInput(encoder), this.angleMotor);

            pidController.setOutputRange(-1, 1);
            pidController.setContinuous();
            pidController.enable(); 
        */
    }

    public void setDrive(double speed, double angle) {
        mPeriodicIO.driveDemand = speed;
        mPeriodicIO.driveAngle = angle;
    }

    @Override
    public void writePeriodicOutputs() {
        mDriveMotor.set(ControlMode.PercentOutput, mPeriodicIO.driveDemand);

        // This is supposed to calculate 'optimization offset'
        double setpoint = mPeriodicIO.driveAngle * (kMaximumVoltage * 0.5) + (kMaximumVoltage * 0.5);
        
        if (setpoint < 0) {
            setpoint = kMaximumVoltage + setpoint;
        }

        if (setpoint > kMaximumVoltage) {
            setpoint = setpoint - kMaximumVoltage;
        }

        // Again, because the constructor in the Gitbook did not work, the rotation motor isn't actually being set anywhere. I will fix this soon
        mPidController.setGoal(setpoint);

        // I also found this online - maybe this could somehow work?
        // mRotationMotor.set(mPidController.calculate(encoder.getDistance(), setpoint));
    }

    @Override
    public void stop() { }

    @Override
    public boolean checkSystem() { return true; }

    @Override
    public void outputTelemetry() { }

    public static class PeriodicIO {
        // OUTPUTS
        public double driveDemand;
        public double driveAngle;
    }
}