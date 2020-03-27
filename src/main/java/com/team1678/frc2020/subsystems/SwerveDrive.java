package com.team1678.frc2020.subsystems;
// Using the instructions from https://jacobmisirian.gitbooks.io/frc-swerve-drive-programming/content/chapter1.html
public class SwerveDrive {
   
    public final double L = 27;
    public final double W = 27;
    // Ensureing there are no errors on the Robot.java
    private WheelJackDrive backRight;
    private WheelJackDrive backLeft;
    private WheelJackDrive frontRight;
    private WheelJackDrive frontLeft;
    
    public SwerveDrive (WheelJackDrive backRight, WheelJackDrive backLeft, WheelJackDrive frontRight, WheelJackDrive frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
    }

    public void drive(double x1, double y1, double x2) {
        double rte = Math.sqrt (Math.pow(L, 2) + Math.pow(W, 2)); // pow being Power of (ex. a^3 is the same as Math.pow(a, 3))
        y1 *= -1;

        double ace = x1 - x2 * (L / rte);
        double bam = x1 + x2 * (L / rte);
        double cut = y1 - x2 * (W / rte);
        double die = y1 + x2 * (W / rte);

        double backRightSpeed = Math.sqrt (Math.pow(ace, 2) + Math.pow(die, 2));
        double backLeftSpeed = Math.sqrt (Math.pow(ace, 2) + Math.pow(cut, 2));
        double frontRightSpeed = Math.sqrt (Math.pow(bam, 2) + Math.pow(die, 2));
        double frontLeftSpeed = Math.sqrt (Math.pow(bam, 2) + Math.pow(cut, 2));

        double backRightAngles = Math.atan2(ace, die) / Math.PI;
        double backLeftAngles = Math.atan2(ace, cut) / Math.PI;
        double frontRightAngles = Math.atan2(bam, die) / Math.PI;
        double frontLeftAngles = Math.atan2(bam, cut) / Math.PI;

        backRight.drive (backRightSpeed, backRightAngles);
        backLeft.drive (backLeftSpeed, backLeftAngles);
        frontRight.drive (frontRightSpeed, frontRightAngles);
        frontLeft.drive (frontLeftSpeed, frontLeftAngles);
    }
}