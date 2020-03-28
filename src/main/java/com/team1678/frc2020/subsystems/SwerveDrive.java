package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.subsystems.Drivetrain;

public class SwerveDrive extends Subsystem {
    // length and width constants
    public final double WHEELBASE = 29.7;
    public final double TRACKWIDTH = 21.6;

    // wheel motors
    private Drivetrain mBackRight;
    private Drivetrain mBackLeft;
    private Drivetrain mFrontRight;
    private Drivetrain mFrontLeft;

    public SwerveDrive(Drivetrain backRight, Drivetrain backLeft, Drivetrain frontRight, Drivetrain frontLeft) {
        this.mBackRight = backRight;
        this.mBackLeft = backLeft;
        this.mFrontRight = frontRight;
        this.mFrontLeft = frontLeft;
    }

    /*
     * @param x1 = x axis of strafing joystick
     * @param y1 = y axis of strafing joystick
     * @param x2 = x axis of rotation joystick
     */

    public void drive(double x1, double y1, double x2) {
        
        // Swerve math
        double diagonal = Math.sqrt(Math.pow(WHEELBASE, 2) + Math.pow(TRACKWIDTH, 2));
        y1 *= -1;

        double a = x1 - x2 * (WHEELBASE / diagonal);
        double b = x1 + x2 * (WHEELBASE / diagonal);
        double c = y1 - x2 * (TRACKWIDTH / diagonal);
        double d = y1 + x2 * (TRACKWIDTH / diagonal);

        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

        double backRightAngle = Math.atan2 (a, d) / Math.PI;
        double backLeftAngle = Math.atan2 (a, c) / Math.PI;
        double frontRightAngle = Math.atan2 (b, d) / Math.PI;
        double frontLeftAngle = Math.atan2 (b, c) / Math.PI;

        // Apply speeds and angles calculations
        mBackRight.drive(backRightSpeed, backRightAngle);
        mBackLeft.drive(backLeftSpeed, backLeftAngle);
        mFrontRight.drive(frontRightSpeed, frontRightAngle);
        mFrontLeft.drive(frontLeftSpeed, frontLeftAngle);
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
