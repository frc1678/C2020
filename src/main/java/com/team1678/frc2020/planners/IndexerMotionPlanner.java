package com.team1678.frc2020.planners;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.subsystems.Indexer;

public class IndexerMotionPlanner {
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
        if (slotNumber < 0) {
            slotNumber += 5;
        } else if (slotNumber > 4) {
            slotNumber -= 5;
        }

        return slotNumber;
    }

    public int findNearestSlotToIntake(double indexer_angle) {
        double wrappedIndexerAngle = WrapDegrees(indexer_angle);

        if (wrappedIndexerAngle < 0) {
            wrappedIndexerAngle += 360;
        }

        int slotNumber = (int) Math.round((wrappedIndexerAngle) / Constants.kAnglePerSlot);
        if (slotNumber < 0) {
            slotNumber += 5;
        } else if (slotNumber > 4) {
            slotNumber -= 5;
        }

        return slotNumber;
    }

    public double findSnappedAngleGoal(double indexer_angle) {
        double wrappedIndexerAngle = WrapDegrees(indexer_angle);

        double angleGoal = wrappedIndexerAngle % 72;

        if (angleGoal > 36) {
            angleGoal -= 72;
        }

        return angleGoal + indexer_angle;
    }

    public boolean isSnapped(double indexer_angle) {
        return Math.abs(indexer_angle % 72) < Constants.kIndexerDeadband;
    }

    public int findNextSlot(double indexer_angle, double turret_angle) {
        int currentSlot = findNearestSlot(indexer_angle, turret_angle);
        int nextSlot;

        if (currentSlot == 4) {
            nextSlot = 0;
        } else {
            nextSlot = currentSlot + 1;
        }

        return nextSlot;
    }

    public int findPreviousSlot(double indexer_angle, double turret_angle) {
        int currentSlot = findNearestSlot(indexer_angle, turret_angle);
        int nextSlot;

        if (currentSlot == 0) {
            nextSlot = 4;
        } else {
            nextSlot = currentSlot - 1;
        }

        return nextSlot;
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

    public boolean isAtGoal(int slotNumber, double indexer_angle, double turret_angle) {
        return Math.abs(findAngleToGoal(slotNumber, indexer_angle, turret_angle)) <= Constants.kIndexerDeadband;
    }

    public int findNearestOpenSlot(double indexer_angle, Indexer.ProxyStatus proxy_status) {
        int currentSlot = findNearestSlotToIntake(indexer_angle);
        int slotGoal;

        boolean frontProxy = proxy_status.front_proxy;
        boolean rightProxy = proxy_status.right_proxy;
        boolean leftProxy = proxy_status.left_proxy;
        boolean backRightProxy = proxy_status.back_right_proxy;
        boolean backLeftProxy = proxy_status.back_left_proxy;
        
        if (!frontProxy) {
            slotGoal = currentSlot;
        } else {
            if (!rightProxy) {
                slotGoal = currentSlot + 1;
            } else {
                if (!leftProxy) {
                    slotGoal = currentSlot - 1;
                } else {
                    if (!backRightProxy) {
                        slotGoal = currentSlot + 2;
                    } else {
                        if (!backLeftProxy) {
                            slotGoal = currentSlot - 2;
                        } else {
                            slotGoal = currentSlot; // all slots filled
                        }
                    }
                }
            }
        }

        if (slotGoal < 0) {
            slotGoal += 5;
        } else if (slotGoal > 4) {
            slotGoal -= 5;
        }

        return slotGoal;
    }
}
