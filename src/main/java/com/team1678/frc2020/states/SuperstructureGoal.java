package com.team1678.frc2020.states;

/**
 * Represents a goal for the superstructure
 */
public class SuperstructureGoal {
    public final SuperstructureState state;

    public SuperstructureGoal(double turret, double hood, double shooter) {
        this(new SuperstructureState(turret, hood, shooter));
    }

    public SuperstructureGoal(SuperstructureState state) {
        this.state = new SuperstructureState(state);
    }

    public boolean equals(SuperstructureGoal other) {
        return this.state.turret == other.state.turret &&
                this.state.hood == other.state.hood &&
                this.state.shooter == other.state.shooter;
    }

    public boolean isAtDesiredState(SuperstructureState currentState) {
        double[] distances = {
                currentState.turret - state.turret,
                currentState.hood - state.hood,
                currentState.shooter - state.shooter,
        };

        for (int i = 0; i < distances.length; i++) {
            if (Math.abs(distances[i]) > SuperstructureConstants.kPadding[i]) {
                return false;
            }
        }

        return true;
    }
}
