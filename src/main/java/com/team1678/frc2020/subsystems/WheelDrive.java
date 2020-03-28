package com.team1678.frc2020.subsystems;

//import edu.wpi.first.wpilibj.PIDController;
import com.team1678.lib.control.PIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class WheelDrive extends Subsystem{
    private TalonFX motorSpeed;
    private PIDController pidController;

    private final double kMaxV = 4.95; // MAX voltage

    public WheelDrive(int motorAngle, int motorSpeed, int encoder) {
        new TalonFX(motorAngle);
        this.motorSpeed = new TalonFX(motorSpeed);

        pidController = new PIDController(1, 0, 0);

        pidController.setOutputRange (-1, 1);
        pidController.setContinuous (false);
        pidController.enable ();
    }

    public void drive(double speed, double angle) {

        motorSpeed.set(ControlMode.PercentOutput, speed); // motorSpeed == speedMotor

        double setpoint = angle * (kMaxV * 0.5) + (kMaxV * 0.5); // Calculate offset
        if (setpoint < 0) {
            setpoint = kMaxV + setpoint;
        }
        if (setpoint > kMaxV) {
            setpoint = setpoint - kMaxV;
        }
        pidController.setGoal(setpoint); // setGoal == setPoint
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