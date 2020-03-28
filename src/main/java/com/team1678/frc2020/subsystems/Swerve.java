package com.team1678.frc2020.subsystems;
import com.team1678.frc2020.Constants;

public class Swerve extends Subsystem {
    // Swerve modules
    private SwerveModule backRight;
    private SwerveModule backLeft;
    private SwerveModule frontRight;
    private SwerveModule frontLeft;
    
    private static PeriodicIO mPeriodicIO = new PeriodicIO();

    private static Swerve mInstance;

    public Swerve() {
        // Insert true values
        backRight = new SwerveModule (0, 1, 0);
        backLeft = new SwerveModule (2, 3, 1);
        frontRight = new SwerveModule (4, 5, 2);
        frontLeft = new SwerveModule (6, 7, 3);
    }

    public synchronized static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    }

    // STR = Strafe Right (x component); FWD = Forward (y component); RCW = Rotate Clockwise
    public synchronized void drive(double STR, double FWD, double RCW) {
        double R = Math.sqrt (Math.pow(Constants.kRobotLength, 2) + Math.pow(Constants.kRobotWidth, 2));

        double A = STR - RCW * (Constants.kRobotLength/R);
        double B = STR + RCW * (Constants.kRobotLength/R);
        double C = FWD - RCW * (Constants.kRobotWidth/R);
        double D = FWD + RCW * (Constants.kRobotWidth/R);

        mPeriodicIO.backRightSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        mPeriodicIO.backLeftSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));
        mPeriodicIO.frontRightSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        mPeriodicIO.frontLeftSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
        
        mPeriodicIO.backRightAngle = Math.atan2 (A, D) / Math.PI;
        mPeriodicIO.backLeftAngle = Math.atan2 (A, C) / Math.PI;
        mPeriodicIO.frontRightAngle = Math.atan2 (B, D) / Math.PI;
        mPeriodicIO.frontLeftAngle = Math.atan2 (B, C) / Math.PI;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        frontRight.setDrive(mPeriodicIO.frontRightSpeed, mPeriodicIO.frontRightAngle);
        frontLeft.setDrive(mPeriodicIO.frontLeftSpeed, mPeriodicIO.frontLeftAngle);
        backRight.setDrive(mPeriodicIO.backRightSpeed, mPeriodicIO.backRightAngle);
        backLeft.setDrive(mPeriodicIO.backLeftSpeed, mPeriodicIO.backLeftAngle);
    }

    @Override
    public synchronized void stop() { }

    @Override
    public boolean checkSystem() { return true; }

    @Override
    public void outputTelemetry() { }

    public static class PeriodicIO {
        // OUTPUTS
        public double frontRightSpeed, frontLeftSpeed, backRightSpeed, backLeftSpeed;
        public double frontRightAngle, frontLeftAngle, backRightAngle, backLeftAngle;
    }
}