package com.team1678.frc2020.planners;

import com.team1678.frc2020.Constants;

public class IndexerMotionPlanner {
    private boolean[] slots = {false, false, false, false, false};
    public IndexerMotionPlanner() {}

    protected double WrapDegrees(double degrees) {
        degrees = degrees % 360.0;
        degrees = (degrees + 360.0) % 360.0;
        if (degrees > 180.0)
            degrees -= 360.0;
        return degrees;
    }

    public int findNearestSlot(double indexer_angle, double turret_angle) {
        double wrappedIndexerAngle = WrapDegrees(indexer_angle);
        double wrappedTurretAngle = WrapDegrees(turret_angle);

        double offset = 0; 
        
        if (Math.abs(wrappedTurretAngle - wrappedIndexerAngle) > 180.0) {
            if (wrappedIndexerAngle > 0.0 && wrappedTurretAngle < 0.0) {
                offset = 360 - wrappedIndexerAngle + wrappedTurretAngle;
            } else if (wrappedIndexerAngle < 0.0 && wrappedTurretAngle > 0.0) {
                offset = -360 - wrappedIndexerAngle + wrappedTurretAngle;
            }
        } else {
            offset = wrappedTurretAngle - wrappedIndexerAngle;
        }

        int slotNumber = (int) Math.round(offset / Constants.kAnglePerSlot);

        return wrapSlotNumber(slotNumber);
    }

    public int findNearestSlotToIntake(double indexer_angle) {
        double wrappedIndexerAngle = WrapDegrees(indexer_angle);

        if (wrappedIndexerAngle < 0) {
            wrappedIndexerAngle += 360;
        }

        int slotNumber = (int) Math.round((wrappedIndexerAngle) / Constants.kAnglePerSlot);

        return wrapSlotNumber(slotNumber);
    }

    public int wrapSlotNumber(int slotNumber) {
        if (slotNumber < 0) {
            slotNumber += 5;
        } else if (slotNumber > 4) {
            slotNumber -= 5;
        }

        return slotNumber;
    }

    public boolean[] updateSlotStatus(double indexer_angle, boolean[] raw_slots) {
        int frontSlot = findNearestSlot(indexer_angle, 0);

        int idx = frontSlot;
        for (int i = 0; i < 5; i++) {
            slots[idx] = raw_slots[i];
            idx++;
            if (idx > 4) {
                idx = 0;
            }
        }

        return slots;
    }

    public double findAngleGoalToIntake(int slotNumber, double indexer_angle) {
        return findAngleGoal(slotNumber, indexer_angle, 0);
    }

    public boolean isSnapped(double indexer_angle) {
        return Math.abs(indexer_angle % 72) < Constants.kIndexerDeadband;
    }

    public int findNextSlot(double indexer_angle, double turret_angle) {
        int currentSlot = findNearestSlot(indexer_angle, turret_angle);

        return wrapSlotNumber(currentSlot + 1);
    }

    public int findPreviousSlot(double indexer_angle, double turret_angle) {
        int currentSlot = findNearestSlot(indexer_angle, turret_angle);

        return wrapSlotNumber(currentSlot - 1);    
    }

    public double findAngleGoal(int slotNumber, double indexer_angle, double turret_angle) {
        return findAngleToGoal(slotNumber, indexer_angle, turret_angle) + indexer_angle;
    }
    
    public double findAngleToGoal(int slotNumber, double indexer_angle, double turret_angle) {
        double wrappedIndexerAngle = WrapDegrees(indexer_angle);
        double wrappedTurretAngle = WrapDegrees(turret_angle);

        double slotAngle = WrapDegrees(slotNumber * Constants.kAnglePerSlot) + wrappedIndexerAngle;

        double angleGoal = WrapDegrees(wrappedTurretAngle - slotAngle);

        return angleGoal;
    }

    public double findNearestDeadSpot(int slotNumber, double indexer_angle, double turret_angle) {
        return findAngleGoal(slotNumber, indexer_angle, turret_angle) + 36.0;
    }

    public boolean isAtGoal(int slotNumber, double indexer_angle, double turret_angle) {
        return Math.abs(findAngleToGoal(slotNumber, indexer_angle, turret_angle)) < Constants.kIndexerDeadband;
    }

    public boolean isAtDeadSpot(int slotNumber, double indexer_angle, double turret_angle) {
        return Math.abs(findNearestDeadSpot(slotNumber, indexer_angle, turret_angle) - 36.0) < Constants.kIndexerDeadband;
    }

    public boolean isAtIntake(int slotNumber, double indexer_angle) {
        return Math.abs(findAngleToGoal(slotNumber, indexer_angle, 0)) < Constants.kIndexerDeadband;
    }

    public int findNearestOpenSlot(double indexer_angle) {
        int currentSlot = findNearestSlot(indexer_angle, 0);
        int idx = currentSlot;
        for (int i = 0; i < 5; i++) {
            if (!slots[idx]) {
                return idx;
            }
            idx++;
            if (idx > 4) {
                idx = 0;
            }
        }

        return currentSlot;
    }
}
