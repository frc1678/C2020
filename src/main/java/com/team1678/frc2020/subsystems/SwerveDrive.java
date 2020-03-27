package com.team1678.frc2020.subsystems;

import java.util.Set;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.motion.SetpointGenerator;
import com.team254.lib.motion.SetpointGenerator.Setpoint;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

public class SwerveDrive extends Subsystem {

    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;

    public SwerveDrive (WheelDrive backRight, WheelDrive backleft, WheelDrive frontright, WheelDrive frontleft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
    }
    public final double L = 27.0; //measurements in thread
    public final double W = 27.0; 
    public void Drive (double x1, double y1, double x2){
        double r = Math.sqrt((L * L) + (W * W));
        y1 *= -1;

        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);


        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (d * d));

        double backRightAngle = Math.atan2 (a, d) / Math.pi;
        double backLeftAngle = Math.atan2 (a, c) / Math.pi;
        double frontRightAngle = Math.atan2 (b, d) / Math.pi;
        double frontLeftAngle = Math.atan2 (b, c) / Math.pi;

        backRight.drive (backRightSpeed, backRightAngle);
        backLeft.drive (backLeftSpeed, backLeftAngle);
        frontRight.drive (frontRightSpeed, frontRightAngle);
        frontLeft.drive (frontLeftSpeed, frontLeftAngle);

    public WheelDrive (int speedMotor, int angleMotor, int encoder) {
        private TalonFX speedMotor;
        private TalonFX angleMotor;
        private PIDController pidcontroller;

        this.speedMotor = new TalonFX (speedMotor);
        this.angleMotor = new TalonFX (angleMotor);
        pidcontroller = new PIDController (1, 0, 0, new AnalogInput (encoder), this.angleMotor);
        
        pidcontroller.setOutputRange (-1, 1);
        pidController.setContinuous ();
        pidController.enable ();

        private final double MAX_VOLTS = 4.95;

        public void drive (double speed, double angle) {
            speedMotor.set (speed);

            double Setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); //Calculate optimization offset 
            if (Setpoint < 0) {
                Setpoint = MAX_VOLTS + Setpoint;
            }
            if (Setpoint > MAX_VOLTS) {
                Setpoint = Setpoint - MAX_VOLTS;
            }
            pidController.setSetpoint (Setpoint);
            
            this.motorSpeed = new TalonFX(motorSpeed);
        
        }
    } 

    }

}