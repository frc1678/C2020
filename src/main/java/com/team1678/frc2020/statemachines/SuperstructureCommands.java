package com.team1678.frc2020.statemachines;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.states.SuperstructureGoal;
import com.team1678.frc2020.states.SuperstructureState;
import com.team1678.frc2020.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

/**
 * Commands for the Superstructure to go to predetermined states or vision based
 * states
 */
public class SuperstructureCommands {

    private SuperstructureCommands() {
    }

    private static void sendCommandToSuperstructure(SuperstructureState position) {
        Superstructure ss = Superstructure.getInstance();
        ss.setGoal(new SuperstructureGoal(position));
    }

    public static void setTurretPosition(double position) {
        Superstructure ss = Superstructure.getInstance();
        SuperstructureGoal lastCommand = ss.getGoal();
        if (lastCommand == null) {
            return;
        }
        SuperstructureState newCommand = new SuperstructureState(position, lastCommand.state.hood,
                lastCommand.state.shooter, lastCommand.state.feed);

        sendCommandToSuperstructure(newCommand);
    }

    public static void setTurretManualHeading(Rotation2d manualHeading) {
        setTurretPosition(Util.toTurretSafeAngleDegrees(manualHeading));
    }

    // Jogs

    public static void jogTurret(double deltaP) {
        Superstructure ss = Superstructure.getInstance();
        ss.jogTurret(deltaP);
    }

    public static void jogHood(double delta) {
        Superstructure ss = Superstructure.getInstance();
        ss.JogHood(delta);
    }

    // shoot shot
    public static void goToShot(SuperstructureState unfed_state) {
        unfed_state.feed = true;
        sendCommandToSuperstructure(unfed_state);
    }

    public static void goToFendorShot() {
        sendCommandToSuperstructure(FendorShot);
    }

    public static void goToFarControlPanelShot() {
        sendCommandToSuperstructure(FarControlPanelShot);
    }

    static SuperstructureState FarControlPanelShot = new SuperstructureState(0, 1.75, -28.0, false);
    static SuperstructureState FendorShot = new SuperstructureState(0, 1.75, -28.0, false);

}
