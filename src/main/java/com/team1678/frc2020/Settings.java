/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2020;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * 
 */
public class Settings {

    private static Settings instance = new Settings(); 

    public static final boolean kIsUsingCompBot = true;
	public static final boolean kIsUsingTractionWheels = true;

    public static final boolean kSimulate = false;
	public static final boolean kResetTalons = false;
    
    // Separate debugging output into the different subsystems so as to not 
    // overload the NetworkTables
    private boolean kDebugSwerve = false;
    private boolean kDebugTurret = false;
    private boolean kDebugShooter = false;
    private boolean kDebugVision = false;

    private NetworkTableEntry swerveToggle;
    private NetworkTableEntry turretToggle;
    private NetworkTableEntry shooterToggle;
    private NetworkTableEntry visionToggle;

    private final String TAB = "Settings";

    private void putToggles() {
        swerveToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Swerve", kDebugSwerve).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        turretToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Turret", kDebugTurret).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        shooterToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Shooter", kDebugShooter).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        visionToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Vision", kDebugVision).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    }

    private void updateSettings() {
        instance.kDebugSwerve = swerveToggle.getBoolean(instance.kDebugSwerve);
        instance.kDebugTurret = turretToggle.getBoolean(instance.kDebugTurret);
        instance.kDebugShooter = shooterToggle.getBoolean(instance.kDebugShooter);
        instance.kDebugVision = visionToggle.getBoolean(instance.kDebugVision);
    }

    public static void initializeToggles() {
        instance.putToggles();
    }

    public static void update() {
        instance.updateSettings();
    }

    public static boolean debugSwerve(){ return instance.kDebugSwerve; }
    public static boolean debugTurret(){ return instance.kDebugTurret; }
    public static boolean debugShooter() { return instance.kDebugShooter; }
    public static boolean debugVision(){ return instance.kDebugVision; }

}
