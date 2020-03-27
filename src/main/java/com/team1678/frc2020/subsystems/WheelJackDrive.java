package com.team1678.frc2020.subsystems;
// Using instructions from https://jacobmisirian.gitbooks.io/frc-swerve-drive-programming/content/chapter1.html
import com.team1678.lib.control.PIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;

public class WheelJackDrive {
    private TalonFX angleMotors;
    private TalonFX angleSpeedy;
    private PIDController pidController;

    public WheelJackDrive(int angleMotors, int angleSpeedy, int encoder) {

        this.angleMotors = new TalonFX (angleMotors);
        this.angleSpeedy = new TalonFX (angleSpeedy);
        // pidController = new PIDController(P, I, D)

    }

    private final double MAX_VOLTS = 4.95; // Used from the site assuming swerve motors are all the same.

    public void drive (double speed, double angle) {
        angleSpeedy.set(ControlMode.PercentOutput, speed);

        double setPoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5);
        if (setPoint < 0) {
            setPoint = MAX_VOLTS + setPoint;
        } 
        if (setPoint > MAX_VOLTS) {
            setPoint = setPoint - MAX_VOLTS;
        }

        pidController.setGoal(setPoint);
    }
}
