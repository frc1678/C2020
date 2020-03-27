package com.team1678.frc2020.subsystems;

public class SwerveDrive extends Subsystem {

    private WheelDrive frontLeft;
    private WheelDrive frontRight;
    private WheelDrive backLeft;
    private WheelDrive backRight;

    public final double L = 27; //Dimention mentioned in the thread
    public final double W = 27;

    public SwerveDrive (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
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

    public void drive(double x1, double y1, double x2) {
        
        double r = Math.sqrt ((L * L) + (W * W));
        y1 *= -1;

        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        double backRightAngle = Math.atan2(a, d) / Math.PI;
        double backLeftAngle = Math.atan2(a, c) / Math.PI;
        double frontRightAngle = Math.atan2(b, d) / Math.PI;
        double frontLeftAngle = Math.atan2(b, c) / Math.PI;

        frontLeft.drive (frontLeftSpeed, frontLeftAngle);
        frontRight.drive (frontRightSpeed, frontRightAngle);
        backLeft.drive (backLeftSpeed, backLeftAngle);
        backRight.drive (backRightSpeed, backRightAngle);
    }
}