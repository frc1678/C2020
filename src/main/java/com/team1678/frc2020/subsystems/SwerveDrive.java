package com.team1678.frc2020.subsystems;

public class SwerveDrive {
    public final double L = 27.0; // length of the robot
    public final double W = 27.0; // width of the robot
    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;
    /*
    * Constructor for swerveDrive
    * 
    * @param backRight
    * @param backLeft
    * @param frontRight
    * @param frontLeft
    * */
    public SwerveDrive (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
    }
    /*
     * @param x1 the x axis of the staffing stick
     * 
     * @param y1 the y axis of the staffing stick
     * 
     * @param x2 the x axis of the rotation stick
     */
    public void drive(double x1, double y1, double x2) {
        double r = Math.sqrt(Math.pow(W, 2) + Math.pow(L, 2)); // Diaganol across opposite robot corners
        y1 *= -1;

        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double backRightSpeed = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));
        double backLeftSpeed = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));
        double frontRightSpeed = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
        double frontLeftSpeed = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));

        double backRightAngle = Math.atan2(a, d) / Math.PI;
        double backLeftAngle = Math.atan2(a, c) / Math.PI;
        double frontRightAngle = Math.atan2(b, d) / Math.PI;
        double frontLeftAngle = Math.atan2(b, c) / Math.PI;

        backRight.drive (backRightSpeed, backRightAngle);
        backLeft.drive (backLeftSpeed, backLeftAngle);
        frontRight.drive (frontRightSpeed, frontRightAngle);
        frontLeft.drive (frontLeftSpeed, frontLeftAngle);
    }
}