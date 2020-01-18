package com.team1678.frc2020.states;

/**
 * Represents a goal for the superstructure
 */
public class SuperstructureGoal {
    public final SuperstructureState state;

    public SuperstructureGoal(double turret, double shooter, double hood) {
        this(new SuperstructureState(turret, shooter, hood));
    }

    public SuperstructureGoal(SuperstructureState state) {
        this.state = new SuperstructureState(state);
    }

    public boolean equals(SuperstructureGoal other) {
        return this.state.turret == other.state.turret && this.state.shooter == other.state.shooter
                && this.state.hood == other.state.hood;
    }

    public boolean isAtDesiredState(SuperstructureState currentState) {
        double[] distances = { currentState.turret - state.turret, currentState.shooter - state.shooter,
                currentState.hood - state.hood };

        for (int i = 0; i < distances.length; i++) {
            if (Math.abs(distances[i]) > SuperstructureConstants.kPadding[i]) {
                return false;
            }
        }

        return true;
    }
}
